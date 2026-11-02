// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>

namespace sharpa::mujoco::plugins::common
{

struct UdpEndpoint
{
    std::string ip = "127.0.0.1";
    std::uint16_t port = 0;
};

enum class UdpBindMode
{
    ReuseAddress,
    Exclusive,
};

class UdpSocket
{
public:
    UdpSocket() = default;
    ~UdpSocket();

    UdpSocket(const UdpSocket&) = delete;
    UdpSocket& operator=(const UdpSocket&) = delete;

    bool open_and_bind(
        const UdpEndpoint& endpoint,
        int receive_timeout_ms,
        std::string* error = nullptr,
        UdpBindMode bind_mode = UdpBindMode::ReuseAddress);
    void close() noexcept;
    bool is_open() const noexcept;
    std::uint16_t local_port(std::string* error = nullptr) const;

    bool send_to(
        const UdpEndpoint& endpoint,
        const std::uint8_t* data,
        std::size_t size,
        std::string* error = nullptr) const;
    bool receive(
        std::uint8_t* data,
        std::size_t capacity,
        std::size_t& received,
        std::string* error = nullptr,
        UdpEndpoint* source = nullptr) const;

private:
    std::atomic<std::intptr_t> handle_{-1};
};

}  // namespace sharpa::mujoco::plugins::common
