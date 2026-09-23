// LowLevelUdpLink -- the transport under the simulated Mini-Pi low-level bus.
//
// Deliberately ROS-free and header-only, so that the SAME code is compiled
// into both ends:
//   - deploy/src/hardware/SimHardware.cpp  (links roscpp)
//   - simulate/src/mini_pi_bridge.cpp      (does not, exactly like Unitree's
//                                           unitree_mujoco)
//
// This is the Mini-Pi stand-in for Unitree's SDK2 ChannelPublisher /
// ChannelSubscriber pair. It is a plain unconnected UDP socket on loopback:
//   - datagram boundaries == packet boundaries, so a partial read is
//     impossible and no framing logic is needed;
//   - a dropped packet is simply a missed cycle, which is what a missed CAN
//     frame is on the real bus, and which the freshness logic above already
//     has to handle;
//   - "latest wins" is the correct semantic for both directions. Neither a
//     stale servo command nor a stale feedback frame has any value, so the
//     receivers drain the socket and keep only the newest datagram. This is
//     the same behaviour as Unitree's latched DDS subscription.
//
// No retry, no reordering, no reliability layer: adding one would make the
// simulated bus BETTER than the physical bus, which would hide exactly the
// class of problem this simulator exists to expose.
#pragma once

#include "hardware/MiniPiLowLevel.h"

#include <arpa/inet.h>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <netinet/in.h>
#include <string>
#include <sys/socket.h>
#include <unistd.h>

namespace mini_pi
{
namespace lowlevel
{

class LowLevelUdpLink
{
public:
    LowLevelUdpLink() = default;
    ~LowLevelUdpLink() { close(); }

    LowLevelUdpLink(const LowLevelUdpLink&) = delete;
    LowLevelUdpLink& operator=(const LowLevelUdpLink&) = delete;

    /// Bind for receiving on `bind_port` and aim sends at `peer_ip:peer_port`.
    /// `bind_port == 0` means "receive-only capability not needed"; the socket
    /// is still created so send() works.
    ///
    /// The socket is non-blocking. Both ends are paced by something else (the
    /// FSM control loop on one side, the physics loop on the other), so
    /// blocking on the bus would couple two clocks that must stay independent.
    bool open(const std::string& peer_ip, int peer_port, int bind_port)
    {
        close();
        fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (fd_ < 0) { error_ = std::string("socket: ") + std::strerror(errno); return false; }

        int flags = ::fcntl(fd_, F_GETFL, 0);
        if (flags < 0 || ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK) < 0)
        {
            error_ = std::string("fcntl O_NONBLOCK: ") + std::strerror(errno);
            close();
            return false;
        }

        // Makes a restart of either process immediate rather than blocked on
        // TIME_WAIT-like states, which matters when the simulator is being
        // restarted repeatedly during a test session.
        int one = 1;
        ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

        if (bind_port > 0)
        {
            sockaddr_in addr{};
            addr.sin_family      = AF_INET;
            addr.sin_addr.s_addr = htonl(INADDR_ANY);
            addr.sin_port        = htons(static_cast<std::uint16_t>(bind_port));
            if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0)
            {
                error_ = "bind port " + std::to_string(bind_port) + ": " +
                         std::strerror(errno);
                close();
                return false;
            }
        }

        std::memset(&peer_, 0, sizeof(peer_));
        peer_.sin_family = AF_INET;
        peer_.sin_port   = htons(static_cast<std::uint16_t>(peer_port));
        if (::inet_pton(AF_INET, peer_ip.c_str(), &peer_.sin_addr) != 1)
        {
            error_ = "bad peer ip '" + peer_ip + "'";
            close();
            return false;
        }

        error_.clear();
        return true;
    }

    void close()
    {
        if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
    }

    bool isOpen() const { return fd_ >= 0; }
    const std::string& lastError() const { return error_; }

    /// Send one packet. A full socket buffer is reported as failure but is not
    /// an error worth retrying -- it means the peer is not draining, which the
    /// peer's own freshness check will notice.
    template <typename Packet>
    bool send(const Packet& pkt)
    {
        if (fd_ < 0) return false;
        const ssize_t n = ::sendto(fd_, &pkt, sizeof(Packet), 0,
                                   reinterpret_cast<const sockaddr*>(&peer_), sizeof(peer_));
        return n == static_cast<ssize_t>(sizeof(Packet));
    }

    /// Drain the socket and keep only the NEWEST well-formed packet of
    /// `expect_type`. Returns true if `out` was written.
    ///
    /// Draining rather than reading one datagram is what keeps the two loops
    /// decoupled: if the controller stalls for 20 cycles, it must resume from
    /// the robot's CURRENT state, not replay 20 old ones.
    template <typename Packet>
    bool receiveLatest(Packet& out, std::uint16_t expect_type,
                       std::size_t* dropped = nullptr, std::size_t* malformed = nullptr)
    {
        if (fd_ < 0) return false;
        Packet tmp;
        bool got = false;
        std::size_t n_drop = 0, n_bad = 0;
        while (true)
        {
            const ssize_t n = ::recv(fd_, &tmp, sizeof(Packet), 0);
            if (n < 0) break;                       // EAGAIN: socket drained
            if (n != static_cast<ssize_t>(sizeof(Packet))) { ++n_bad; continue; }
            if (!header_ok(tmp.h, expect_type))     { ++n_bad; continue; }
            if (got) ++n_drop;                      // superseded before we saw it
            out = tmp;
            got = true;
        }
        if (dropped)   *dropped   += n_drop;
        if (malformed) *malformed += n_bad;
        return got;
    }

private:
    int fd_ = -1;
    sockaddr_in peer_{};
    std::string error_;
};

} // namespace lowlevel
} // namespace mini_pi
