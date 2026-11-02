// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#include "sharpa_mujoco/plugins/common/socketcan_transport.h"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <utility>

namespace sharpa::mujoco::plugins::common
{
namespace
{

bool socket_error(const char* operation, std::string* error)
{
    if (error) *error = std::string(operation) + ": " + std::strerror(errno);
    return false;
}

bool interface_error(const std::string& name, const std::string& detail, std::string* error)
{
    if (error)
    {
        *error = "SocketCAN interface '" + name + "' " + detail + ". ";
        const bool safe_virtual_name = name.rfind("vcan", 0) == 0 &&
            name.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_.-") == std::string::npos;
        if (safe_virtual_name)
        {
            *error += "The vcan driver can be installed while its interfaces are absent or down. "
                "From the sharpa_mujoco runtime directory run: sudo bash scripts/setup_vcan.sh " + name +
                "; then restart simulation. Launchers can prepare all configured buses using "
                "bash scripts/prepare_environment.sh --config /path/to/sharpa_mujoco.conf.";
        }
        else
        {
            *error += "Prepare the configured CAN interface before starting simulation. "
                "Physical CAN interfaces are never created or reconfigured automatically.";
        }
    }
    return false;
}

SocketCanInterfaceState inspect_interface_on_socket(
    int descriptor,
    const std::string& interface_name,
    int* interface_index,
    std::string* detail)
{
    if (interface_name.empty() || interface_name.size() >= IFNAMSIZ)
    {
        if (detail) *detail = "name is empty or too long";
        return SocketCanInterfaceState::Unusable;
    }

    ifreq request{};
    std::strncpy(request.ifr_name, interface_name.c_str(), IFNAMSIZ - 1);
    if (::ioctl(descriptor, SIOCGIFINDEX, &request) != 0)
    {
        const int saved_errno = errno;
        if (detail)
        {
            *detail = std::string("cannot be resolved (SIOCGIFINDEX: ") +
                std::strerror(saved_errno) + ")";
        }
        return saved_errno == ENODEV
            ? SocketCanInterfaceState::Missing
            : SocketCanInterfaceState::Unusable;
    }
    const int resolved_index = request.ifr_ifindex;

    if (::ioctl(descriptor, SIOCGIFFLAGS, &request) != 0)
    {
        const int saved_errno = errno;
        if (detail)
        {
            *detail = std::string("flags cannot be read: ") +
                std::strerror(saved_errno);
        }
        return SocketCanInterfaceState::Unusable;
    }
    if ((request.ifr_flags & IFF_UP) == 0)
    {
        if (detail) *detail = "is DOWN";
        return SocketCanInterfaceState::Down;
    }

    sockaddr_can address{};
    address.can_family = AF_CAN;
    address.can_ifindex = resolved_index;
    if (::bind(descriptor, reinterpret_cast<const sockaddr*>(&address),
               sizeof(address)) != 0)
    {
        const int saved_errno = errno;
        if (detail)
        {
            *detail = std::string("cannot be bound as SocketCAN: ") +
                std::strerror(saved_errno);
        }
        return SocketCanInterfaceState::Unusable;
    }

    if (interface_index) *interface_index = resolved_index;
    if (detail) detail->clear();
    return SocketCanInterfaceState::Ready;
}

std::uint32_t normalized_can_id(canid_t can_id)
{
    return (can_id & CAN_EFF_FLAG) != 0
        ? static_cast<std::uint32_t>(can_id & CAN_EFF_MASK)
        : static_cast<std::uint32_t>(can_id & CAN_SFF_MASK);
}

bool decode_received_frame(
    const canfd_frame& wire,
    ssize_t received,
    CanFdFrame& frame,
    std::string* error)
{
    if ((wire.can_id & CAN_ERR_FLAG) != 0) return false;

    CanFdFrame decoded;
    decoded.can_id = normalized_can_id(wire.can_id);
    if (received == CAN_MTU)
    {
        const auto* classic = reinterpret_cast<const can_frame*>(&wire);
        decoded.data.assign(
            classic->data,
            classic->data + std::min<std::size_t>(classic->can_dlc, CAN_MAX_DLEN));
    }
    else if (received == CANFD_MTU && wire.len <= CANFD_MAX_DLEN)
    {
        decoded.can_flags = (wire.flags & CANFD_BRS) != 0 ? kCanFdFlagBrs : 0;
        decoded.data.assign(wire.data, wire.data + wire.len);
    }
    else
    {
        if (error) *error = "SocketCAN returned an invalid frame size.";
        return false;
    }
    frame = std::move(decoded);
    return true;
}

}  // namespace

SocketCanInterfaceState inspect_socketcan_interface(
    const std::string& interface_name,
    std::string* detail)
{
    const int descriptor = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (descriptor < 0)
    {
        if (detail) *detail = std::string("socket(PF_CAN): ") + std::strerror(errno);
        return SocketCanInterfaceState::Unusable;
    }
    const SocketCanInterfaceState state =
        inspect_interface_on_socket(descriptor, interface_name, nullptr, detail);
    ::close(descriptor);
    return state;
}

SocketCanFdSocket::~SocketCanFdSocket()
{
    close();
}

bool SocketCanFdSocket::open_and_bind(
    const std::string& interface_name,
    int receive_timeout_ms,
    std::string* error)
{
    close();
    if (interface_name.empty() || interface_name.size() >= IFNAMSIZ)
    {
        if (error) *error = "SocketCAN interface name is empty or too long.";
        return false;
    }
    if (receive_timeout_ms < 0)
    {
        if (error) *error = "SocketCAN receive timeout must not be negative.";
        return false;
    }

    const int descriptor = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (descriptor < 0) return socket_error("socket(PF_CAN)", error);

    const int enable_can_fd = 1;
    if (::setsockopt(descriptor, SOL_CAN_RAW, CAN_RAW_FD_FRAMES,
                     &enable_can_fd, sizeof(enable_can_fd)) != 0)
    {
        const int saved_errno = errno;
        ::close(descriptor);
        errno = saved_errno;
        return socket_error("setsockopt(CAN_RAW_FD_FRAMES)", error);
    }
    // A raw CAN socket normally receives its own transmitted frames. The
    // emulator must only process Platform commands, not its feedback/SDO
    // responses looped back by SocketCAN.
    const int receive_own_messages = 0;
    if (::setsockopt(descriptor, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS,
                     &receive_own_messages, sizeof(receive_own_messages)) != 0)
    {
        const int saved_errno = errno;
        ::close(descriptor);
        errno = saved_errno;
        return socket_error("setsockopt(CAN_RAW_RECV_OWN_MSGS)", error);
    }

    timeval timeout{};
    timeout.tv_sec = receive_timeout_ms / 1000;
    timeout.tv_usec = (receive_timeout_ms % 1000) * 1000;
    if (::setsockopt(descriptor, SOL_SOCKET, SO_RCVTIMEO,
                     &timeout, sizeof(timeout)) != 0)
    {
        const int saved_errno = errno;
        ::close(descriptor);
        errno = saved_errno;
        return socket_error("setsockopt(SO_RCVTIMEO)", error);
    }

    int interface_index = 0;
    std::string interface_detail;
    const SocketCanInterfaceState interface_state = inspect_interface_on_socket(
        descriptor, interface_name, &interface_index, &interface_detail);
    if (interface_state != SocketCanInterfaceState::Ready)
    {
        ::close(descriptor);
        return interface_error(interface_name, interface_detail, error);
    }

    socket_.store(descriptor);
    return true;
}

void SocketCanFdSocket::close() noexcept
{
    const int descriptor = socket_.exchange(-1);
    if (descriptor >= 0)
    {
        ::shutdown(descriptor, SHUT_RDWR);
        ::close(descriptor);
    }
}

bool SocketCanFdSocket::is_open() const noexcept
{
    return socket_.load() >= 0;
}

bool SocketCanFdSocket::send(const CanFdFrame& frame, std::string* error) const
{
    const int descriptor = socket_.load();
    if (descriptor < 0)
    {
        if (error) *error = "SocketCAN socket is closed.";
        return false;
    }
    if (frame.data.size() > CANFD_MAX_DLEN)
    {
        if (error) *error = "SocketCAN FD payload exceeds 64 bytes.";
        return false;
    }

    canfd_frame wire{};
    wire.can_id = frame.can_id;
    wire.len = static_cast<__u8>(frame.data.size());
    wire.flags = CANFD_FDF;
    if ((frame.can_flags & kCanFdFlagBrs) != 0) wire.flags |= CANFD_BRS;
    if (!frame.data.empty())
    {
        std::memcpy(wire.data, frame.data.data(), frame.data.size());
    }
    const ssize_t written = ::write(descriptor, &wire, CANFD_MTU);
    if (written != CANFD_MTU) return socket_error("write(CAN-FD)", error);
    return true;
}

bool SocketCanFdSocket::receive(CanFdFrame& frame, std::string* error) const
{
    const int descriptor = socket_.load();
    if (descriptor < 0)
    {
        if (error) *error = "SocketCAN socket is closed.";
        return false;
    }

    canfd_frame wire{};
    const ssize_t received = ::read(descriptor, &wire, CANFD_MTU);
    if (received < 0)
    {
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) return false;
        return socket_error("read(CAN)", error);
    }
    return decode_received_frame(wire, received, frame, error);
}

bool SocketCanFdSocket::try_receive(
    CanFdFrame& frame,
    std::string* error) const
{
    const int descriptor = socket_.load();
    if (descriptor < 0)
    {
        if (error) *error = "SocketCAN socket is closed.";
        return false;
    }

    canfd_frame wire{};
    const ssize_t received =
        ::recv(descriptor, &wire, CANFD_MTU, MSG_DONTWAIT);
    if (received < 0)
    {
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) return false;
        return socket_error("recv(CAN)", error);
    }
    return decode_received_frame(wire, received, frame, error);
}

}  // namespace sharpa::mujoco::plugins::common
