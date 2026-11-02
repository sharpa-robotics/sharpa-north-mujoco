// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#include "sharpa_mujoco/plugins/common/udp_transport.h"

#include <limits>
#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

namespace sharpa::mujoco::plugins::common
{
namespace
{

using NativeSocket = int;
using SocketLength = socklen_t;
constexpr NativeSocket kInvalidSocket = -1;

bool initialize_sockets(std::string*)
{
    return true;
}

int last_socket_error()
{
    return errno;
}

void close_socket(NativeSocket socket)
{
    ::close(socket);
}

bool is_timeout_error(int error)
{
    return error == EAGAIN || error == EWOULDBLOCK || error == EINTR;
}

NativeSocket to_native(std::intptr_t value)
{
    return static_cast<NativeSocket>(value);
}

std::intptr_t from_native(NativeSocket value)
{
    return static_cast<std::intptr_t>(value);
}

bool make_address(const UdpEndpoint& endpoint, sockaddr_in& address, std::string* error)
{
    address = {};
    address.sin_family = AF_INET;
    address.sin_port = htons(endpoint.port);
    if (inet_pton(AF_INET, endpoint.ip.c_str(), &address.sin_addr) != 1)
    {
        if (error)
        {
            *error = "Invalid IPv4 address: " + endpoint.ip;
        }
        return false;
    }
    return true;
}

bool socket_fail(const std::string& operation, int code, std::string* error)
{
    if (error)
    {
        *error = operation + " failed with socket error " + std::to_string(code);
    }
    return false;
}

}  // namespace

UdpSocket::~UdpSocket()
{
    close();
}

bool UdpSocket::open_and_bind(
    const UdpEndpoint& endpoint,
    int receive_timeout_ms,
    std::string* error,
    UdpBindMode bind_mode)
{
    close();
    if (receive_timeout_ms < 0)
    {
        if (error)
        {
            *error = "UDP receive timeout must not be negative.";
        }
        return false;
    }
    if (!initialize_sockets(error))
    {
        return false;
    }

    sockaddr_in address{};
    if (!make_address(endpoint, address, error))
    {
        return false;
    }
    const NativeSocket socket = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (socket == kInvalidSocket)
    {
        return socket_fail("socket", last_socket_error(), error);
    }

    if (bind_mode == UdpBindMode::ReuseAddress)
    {
        const int reuse = 1;
        setsockopt(
            socket,
            SOL_SOCKET,
            SO_REUSEADDR,
            reinterpret_cast<const char*>(&reuse),
            static_cast<SocketLength>(sizeof(reuse)));
    }

    timeval timeout{};
    timeout.tv_sec = receive_timeout_ms / 1000;
    timeout.tv_usec = (receive_timeout_ms % 1000) * 1000;
    const int timeout_result =
        setsockopt(socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    if (timeout_result != 0)
    {
        const int code = last_socket_error();
        close_socket(socket);
        return socket_fail("setsockopt(SO_RCVTIMEO)", code, error);
    }

    if (::bind(
            socket,
            reinterpret_cast<const sockaddr*>(&address),
            static_cast<SocketLength>(sizeof(address))) != 0)
    {
        const int code = last_socket_error();
        close_socket(socket);
        return socket_fail("bind", code, error);
    }
    handle_.store(from_native(socket));
    return true;
}

void UdpSocket::close() noexcept
{
    const std::intptr_t value = handle_.exchange(-1);
    if (value != -1)
    {
        const NativeSocket socket = to_native(value);
        shutdown(socket, SHUT_RDWR);
        close_socket(socket);
    }
}

bool UdpSocket::is_open() const noexcept
{
    return handle_.load() != -1;
}

std::uint16_t UdpSocket::local_port(std::string* error) const
{
    const std::intptr_t value = handle_.load();
    if (value == -1)
    {
        if (error)
        {
            *error = "UDP socket is closed.";
        }
        return 0;
    }

    sockaddr_in address{};
    SocketLength address_size = static_cast<SocketLength>(sizeof(address));
    if (::getsockname(
            to_native(value),
            reinterpret_cast<sockaddr*>(&address),
            &address_size) != 0)
    {
        socket_fail("getsockname", last_socket_error(), error);
        return 0;
    }
    return ntohs(address.sin_port);
}

bool UdpSocket::send_to(
    const UdpEndpoint& endpoint,
    const std::uint8_t* data,
    std::size_t size,
    std::string* error) const
{
    if (!data || size == 0 || size > static_cast<std::size_t>(std::numeric_limits<int>::max()))
    {
        if (error)
        {
            *error = "Invalid UDP datagram buffer.";
        }
        return false;
    }
    const std::intptr_t value = handle_.load();
    if (value == -1)
    {
        if (error)
        {
            *error = "UDP socket is closed.";
        }
        return false;
    }
    sockaddr_in address{};
    if (!make_address(endpoint, address, error))
    {
        return false;
    }
    const int sent = ::sendto(
        to_native(value),
        reinterpret_cast<const char*>(data),
        static_cast<int>(size),
        0,
        reinterpret_cast<const sockaddr*>(&address),
        static_cast<SocketLength>(sizeof(address)));
    if (sent != static_cast<int>(size))
    {
        return socket_fail("sendto", last_socket_error(), error);
    }
    return true;
}

bool UdpSocket::receive(
    std::uint8_t* data,
    std::size_t capacity,
    std::size_t& received,
    std::string* error,
    UdpEndpoint* source) const
{
    received = 0;
    if (!data || capacity == 0 ||
        capacity > static_cast<std::size_t>(std::numeric_limits<int>::max()))
    {
        if (error)
        {
            *error = "Invalid UDP receive buffer.";
        }
        return false;
    }
    const std::intptr_t value = handle_.load();
    if (value == -1)
    {
        return false;
    }
    sockaddr_in source_address{};
    SocketLength source_size = static_cast<SocketLength>(sizeof(source_address));
    const int result = ::recvfrom(
        to_native(value),
        reinterpret_cast<char*>(data),
        static_cast<int>(capacity),
        0,
        reinterpret_cast<sockaddr*>(&source_address),
        &source_size);
    if (result < 0)
    {
        const int code = last_socket_error();
        if (is_timeout_error(code))
        {
            return false;
        }
        return socket_fail("recvfrom", code, error);
    }
    received = static_cast<std::size_t>(result);
    if (source != nullptr)
    {
        char ip[INET_ADDRSTRLEN] = {};
        if (inet_ntop(AF_INET, &source_address.sin_addr, ip, sizeof(ip)) == nullptr)
        {
            return socket_fail("inet_ntop", last_socket_error(), error);
        }
        source->ip = ip;
        source->port = ntohs(source_address.sin_port);
    }
    return true;
}

}  // namespace sharpa::mujoco::plugins::common
