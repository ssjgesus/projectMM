#pragma once

#include "light/effects/EffectBase.h"

#include "light/ArtNetPacket.h"   // shared ArtNet wire formats (build + parse)
#include "light/DdpPacket.h"      // shared DDP wire format
#include "light/E131Packet.h"     // shared E1.31/sACN wire format
#include "platform/platform.h"    // platform::UdpSocket (the receive sockets — not a scratch buffer)

namespace mm {

// Lights-over-UDP receiver as an EFFECT: external light data is just another
// module that writes into the layer buffer, composable with modifiers and
// blending like any generated effect. The end-to-end pair with
// NetworkSendDriver — and the receive side for industry senders (Resolume,
// Madrix, xLights, LedFx, …).
//
// All three protocols are received AT ONCE: the effect binds the three
// well-known ports (ArtNet 6454, E1.31 5568, DDP 4048) and validates each
// packet against its port's wire format — WLED's multi-port pattern. There is
// deliberately no protocol control: whatever a sender speaks just works, and
// the status field shows what is being received.
//
// ArtNet discovery: controllers (Resolume's Advanced Output, Madrix, xLights)
// find nodes by broadcasting ArtPoll; this effect answers with ArtPollReply so
// the device appears in their node lists instead of needing manual IP entry.
//
// Packets are drained into an owned STAGING buffer and staging is copied to the
// layer buffer each tick — hold-last-frame semantics. DDP additionally assembles
// packets in a private back buffer and publishes to staging only on Push, so a
// render tick can never expose a half-received frame to Preview/PARLIO/i80/RMT.
// If Push is lost, offset zero starts a fresh assembly while the last complete
// frame remains visible. The drain is non-blocking and bounded per tick (network
// input is synchronous at the frame boundary — the architecture.md rule), so a
// packet flood can't wedge the render loop. Sequence fields are ignored.
//
// Prior art: MoonLight's D_NetworkIn (single node, three protocols), WLED's
// realtime UDP input (multi-port + per-packet validation, ArtPollReply), and
// projectMM v1's ArtNetInModule.
// Author: projectMM original (E1.31 / Art-Net receive)
/// Effect that paints the layer from received Art-Net/E1.31/DDP pixels.
class NetworkReceiveEffect : public EffectBase {
public:
    const char* tags() const override { return "📡🌙"; }  // network input · MoonLight / v1 lineage

    uint16_t universeStart = 0;        // mirrors the sender's universe_start (ArtNet/E1.31)
    // Bytes each universe maps to in the buffer. 510 = whole RGB lights per
    // universe (the xLights/Falcon convention and our sender's split); set 512
    // for senders that pack pixels across universe boundaries (Madrix-style).
    // Also clamps the copied payload, so a 512-channel frame from a 510-packed
    // source can't bleed its 2 padding bytes into the next universe's data.
    uint16_t channelsPerUniverse = static_cast<uint16_t>(MAX_CHANNELS_PER_UNIVERSE);

    void defineControls() override {
        controls_.addUint16("universe_start", universeStart);
        controls_.addUint16("channels_per_universe", channelsPerUniverse);
    }

    void release() override {
        // Close the sockets (this effect's own non-buffer state), then chain to the base — which
        // frees the registered staging_ ScratchBuffer (and recurses to children). Chaining is
        // required: without MoonModule::release() the staging buffer would leak on disable.
        artnetSocket_.close();
        e131Socket_.close();
        ddpSocket_.close();
        resetDdpState();
        MoonModule::release();
        clearStatus();
    }

    /// Pure build (see MoonModule::prepare): open + bind the three receive sockets (each bind
    /// independent so one taken port can't stop the others) and size the staging buffer to the layer
    /// (one byte per channel byte, zeroed so a fresh grid starts dark). No enabled() check — core's
    /// applyState() calls this only when effectively-enabled and routes to release() (sockets closed,
    /// staging freed) otherwise, so a disabled effect (or one under a disabled parent) frees the ports.
    void prepare() override {
        const bool artnetOk = artnetSocket_.open() && artnetSocket_.bind(ARTNET_PORT);
        const bool e131Ok = e131Socket_.open() && e131Socket_.bind(E131_PORT);
        const bool ddpOk = ddpSocket_.open() && ddpSocket_.bind(DDP_PORT);
        if (artnetOk && e131Ok && ddpOk) {
            if (status() == kBindFailMsg) clearStatus();
        } else {
            setStatus(kBindFailMsg, Severity::Error);
        }
        // Size the published staging buffer and DDP's private assembly buffer to
        // the layer (one byte per channel byte). Both are allocated off the hot
        // path and automatically accounted/freed by ScratchBuffer.
        const size_t bytes = static_cast<size_t>(nrOfLights()) * channelsPerLight();
        staging_.resize(bytes);
        ddpAssembly_.resize(bytes);
        resetDdpState();
    }

    void tick() override {
        if (!staging_) return;
        // Bounded non-blocking drain per socket: 128 packets ≈ one full ArtNet
        // frame for ~21k RGB lights; a flood costs at most 3×128 recvfrom calls
        // per tick, then the rest waits in the socket buffers for the next tick.
        uint16_t universe = 0, dataLen = 0;
        uint32_t byteOffset = 0;
        const uint8_t* data = nullptr;
        uint8_t srcIp[4];
        for (int i = 0; i < kMaxPacketsPerTick; i++) {
            const int n = artnetSocket_.recvFrom(pkt_, sizeof(pkt_), srcIp);
            if (n <= 0) break;
            if (parseArtDmxPacket(pkt_, static_cast<size_t>(n), universe, data, dataLen)) {
                applyDmx(universe, data, dataLen);
                noteReceiving("Art-Net", srcIp);
            } else if (isArtPoll(pkt_, static_cast<size_t>(n))) {
                replyToPoll(srcIp);   // make the device show up in controller node lists
            }
        }
        for (int i = 0; i < kMaxPacketsPerTick; i++) {
            const int n = e131Socket_.recvFrom(pkt_, sizeof(pkt_), srcIp);
            if (n <= 0) break;
            if (parseE131Packet(pkt_, static_cast<size_t>(n), universe, data, dataLen)) {
                applyDmx(universe, data, dataLen);
                noteReceiving("E1.31", srcIp);
            }
        }
        for (int i = 0; i < kMaxPacketsPerTick; i++) {
            const int n = ddpSocket_.recvFrom(pkt_, sizeof(pkt_), srcIp);
            if (n <= 0) break;
            if (parseDdpPacket(pkt_, static_cast<size_t>(n), byteOffset, data, dataLen)) {
                applyDdp(byteOffset, data, dataLen, (pkt_[0] & DDP_FLAG_PUSH) != 0);
                noteReceiving("DDP", srcIp);
            }
        }
        // Staging → layer buffer (the layer cleared it at tick start).
        uint8_t* buf = buffer();
        if (!buf) return;
        const size_t bufBytes = static_cast<size_t>(nrOfLights()) * channelsPerLight();
        std::memcpy(buf, staging_.data(), staging_.bytes() < bufBytes ? staging_.bytes() : bufBytes);
    }

    // Place one universe's payload: byte offset (universe − universeStart) ×
    // channels_per_universe, payload clamped to one universe's stride so a
    // 512-channel frame can't bleed past its slot. Universes below the start or
    // beyond the buffer are ignored. Shared by ArtNet and E1.31; DDP skips the
    // universe math and calls applyBytes directly. Public for testability (the
    // buildArtDmxPacket precedent).
    void applyDmx(uint16_t universe, const uint8_t* data, uint16_t len) {
        if (universe < universeStart || channelsPerUniverse == 0) return;
        if (len > channelsPerUniverse) len = channelsPerUniverse;
        applyBytes(static_cast<size_t>(universe - universeStart) * channelsPerUniverse,
                   data, len);
    }

    // Apply one DDP packet to the PRIVATE assembly buffer. The published staging
    // buffer is changed only when Push completes the frame, which makes the
    // handoff atomic from every output driver's point of view. Offset zero while
    // a frame is already open abandons that incomplete frame (lost Push/packet)
    // and starts clean while staging keeps showing the last complete frame.
    //
    // `ddpSawZero_` preserves a reordered first packet: if a non-zero offset
    // arrives before offset zero, the later zero belongs to the same assembly
    // instead of falsely discarding it.
    void applyDdp(uint32_t byteOffset, const uint8_t* data, uint16_t len, bool push) {
        if (!staging_ || !ddpAssembly_) return;
        const size_t offset = static_cast<size_t>(byteOffset);
        if (!ddpFrameOpen_ || (offset == 0 && ddpSawZero_)) beginDdpFrame();
        if (offset == 0) ddpSawZero_ = true;

        if (offset < ddpAssembly_.bytes()) {
            size_t n = len;
            const size_t available = ddpAssembly_.bytes() - offset;
            if (n > available) n = available;
            if (n > 0) {
                std::memcpy(ddpAssembly_.data() + offset, data, n);
                const size_t end = offset + n;
                if (ddpAssemblyEnd_ == ddpAssemblyStart_) {
                    ddpAssemblyStart_ = offset;
                    ddpAssemblyEnd_ = end;
                } else {
                    if (offset < ddpAssemblyStart_) ddpAssemblyStart_ = offset;
                    if (end > ddpAssemblyEnd_) ddpAssemblyEnd_ = end;
                }
            }
        }
        if (push) publishDdpFrame();
    }

    // The one clamped direct write into published staging, used by the
    // universe protocols and placement tests. DDP uses the same byte addressing
    // against ddpAssembly_ above, then publishes only at a complete-frame
    // boundary. The bound check runs BEFORE any addition so a hostile offset
    // can't overflow past it.
    void applyBytes(size_t offset, const uint8_t* data, uint16_t len) {
        if (!staging_ || offset >= staging_.bytes()) return;
        size_t n = len;
        if (offset + n > staging_.bytes()) n = staging_.bytes() - offset;
        std::memcpy(staging_.data() + offset, data, n);
    }

    // Test-only accessors — let the unit tests pin the staging lifecycle
    // (sized off the hot path, never reallocated by loop, freed on release).
    const uint8_t* stagingData() const { return staging_.data(); }
    size_t stagingBytes() const { return staging_.bytes(); }

private:
    static constexpr int kMaxPacketsPerTick = 128;
    static constexpr const char* kBindFailMsg = "UDP bind failed — port in use?";
    // The receiving-status buffer: "receiving <protocol> from <ip>", the ONE writable status this effect
    // shows — the same field reports the protocol AND the sender IP, so it answers "who is driving me".
    // setStatus holds the pointer (doesn't copy), so the string must outlive the call, hence a member, not
    // a stack buffer. Sized for the longest form ("receiving Art-Net from 255.255.255.255" = 38 chars +
    // NUL). A bind error uses its own static literal. lastProto_/lastIp_ cache the last-formatted source so
    // noteReceiving can early-out on the common case (same sender + protocol) without even formatting.
    char recvStatus_[40] = "";
    const char* lastProto_ = nullptr;   // last protocol pointer the status was built from (literal, stable)
    uint8_t     lastIp_[4] = {};        // last sender IP the status was built from

    platform::UdpSocket artnetSocket_;
    platform::UdpSocket e131Socket_;
    platform::UdpSocket ddpSocket_;
    uint8_t pkt_[1500] = {};       // one datagram, any protocol (DDP max 1450)
    // Layer-buffer-sized staging (hold-last-frame). Self-sizing, self-freeing, self-reporting;
    // freed on disable via MoonModule::release() (the release() override chains to it).
    ScratchBuffer<uint8_t> staging_{*this};
    // DDP writes here until Push, never into staging_. This second full-size
    // buffer is the isolation boundary that prevents packet-level tearing.
    ScratchBuffer<uint8_t> ddpAssembly_{*this};
    size_t ddpAssemblyStart_ = 0;    // inclusive range written in the frame being assembled
    size_t ddpAssemblyEnd_ = 0;      // exclusive; equal to start means no assembled bytes
    size_t ddpPublishedStart_ = 0;   // inclusive range owned by the last published DDP frame
    size_t ddpPublishedEnd_ = 0;     // exclusive; cleared when the next complete frame publishes
    bool ddpFrameOpen_ = false;
    bool ddpSawZero_ = false;

    void resetDdpState() {
        ddpAssemblyStart_ = 0;
        ddpAssemblyEnd_ = 0;
        ddpPublishedStart_ = 0;
        ddpPublishedEnd_ = 0;
        ddpFrameOpen_ = false;
        ddpSawZero_ = false;
    }

    // Start assembling a new frame from black. Clearing the back buffer is
    // deliberately done once per DDP frame, not once per render tick. It makes
    // missing packets black in the eventual completed frame instead of leaking
    // bytes from any older assembly.
    void beginDdpFrame() {
        std::memset(ddpAssembly_.data(), 0, ddpAssembly_.bytes());
        ddpAssemblyStart_ = 0;
        ddpAssemblyEnd_ = 0;
        ddpFrameOpen_ = true;
        ddpSawZero_ = false;
    }

    // Commit one complete DDP frame to the hold-last published buffer. This
    // runs inside the effect tick before staging is copied to the layer, so no
    // driver can observe the clear and copy as separate states.
    void publishDdpFrame() {
        if (!ddpFrameOpen_) return;
        // A syntactically valid packet can still address wholly beyond this
        // device's buffer. Do not let its Push black a valid displayed frame.
        if (ddpAssemblyEnd_ == ddpAssemblyStart_) {
            ddpFrameOpen_ = false;
            ddpSawZero_ = false;
            return;
        }
        if (ddpPublishedEnd_ > ddpPublishedStart_) {
            std::memset(staging_.data() + ddpPublishedStart_, 0,
                        ddpPublishedEnd_ - ddpPublishedStart_);
        }
        if (ddpAssemblyEnd_ > ddpAssemblyStart_) {
            std::memcpy(staging_.data() + ddpAssemblyStart_,
                        ddpAssembly_.data() + ddpAssemblyStart_,
                        ddpAssemblyEnd_ - ddpAssemblyStart_);
        }
        ddpPublishedStart_ = ddpAssemblyStart_;
        ddpPublishedEnd_ = ddpAssemblyEnd_;
        ddpFrameOpen_ = false;
        ddpSawZero_ = false;
    }

    // Update the "receiving <protocol> from <ip>" diagnostic, but never clobber a bind error (its status is
    // the kBindFailMsg literal, so status() != recvStatus_). The common case is the same sender + protocol
    // every packet, so short-circuit on the cached lastProto_/lastIp_ FIRST — no formatting at all — and
    // only snprintf + setStatus when the source actually changes. proto is always one of the same three
    // string literals, so a pointer compare identifies it. This runs on the receive hot path.
    void noteReceiving(const char* proto, const uint8_t ip[4]) {
        const char* s = status();
        if (s != nullptr && s != recvStatus_) return;   // a bind error (or foreign status) wins
        // Already showing this exact source? nothing to do (skip the format + the setStatus).
        if (s == recvStatus_ && proto == lastProto_ && std::memcmp(ip, lastIp_, 4) == 0) return;
        lastProto_ = proto;
        std::memcpy(lastIp_, ip, 4);
        std::snprintf(recvStatus_, sizeof(recvStatus_), "receiving %s from %u.%u.%u.%u", proto,
                      static_cast<unsigned>(ip[0]), static_cast<unsigned>(ip[1]),
                      static_cast<unsigned>(ip[2]), static_cast<unsigned>(ip[3]));
        setStatus(recvStatus_, Severity::Status);
    }

    // Answer an ArtPoll with our IP/MAC/name so controllers list the device.
    // Runs at most once per poll (controllers poll every few seconds) — the
    // 239-byte reply lives on the stack, no allocation.
    void replyToPoll(const uint8_t pollerIp[4]) {
        uint8_t myIp[4];
        platform::ethGetIPv4(myIp);
        if (!myIp[0] && !myIp[1] && !myIp[2] && !myIp[3]) platform::wifiStaGetIPv4(myIp);
        // Desktop has no eth/wifi netif (both return 0.0.0.0); hostIp() reports the
        // host's LAN IP as a string, so parse that as the last resort.
        if (!myIp[0] && !myIp[1] && !myIp[2] && !myIp[3]) {
            if (!parseDottedQuad(platform::hostIp(), myIp)) return;  // no usable IP — stay silent
        }
        uint8_t mac[6];
        platform::getMacAddress(mac);
        uint8_t reply[ARTNET_POLL_REPLY_SIZE];
        buildArtPollReply(reply, myIp, mac, "projectMM", "projectMM NetworkReceive",
                          universeStart);
        artnetSocket_.sendToAddr(pollerIp, ARTNET_PORT, reply, sizeof(reply));
    }
};

} // namespace mm
