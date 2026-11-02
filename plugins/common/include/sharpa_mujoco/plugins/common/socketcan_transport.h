// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "sharpa_mujoco/plugins/common/mcu_udp_codec.h"

#include <atomic>
#include <string>

namespace sharpa::mujoco::plugins::common
{

enum class SocketCanInterfaceState
{
    Ready,
    Missing,
    Down,
    Unusable,
};

// Read-only probe shared by launch preflight and the transport itself.  It
// never creates or changes an interface.
SocketCanInterfaceState inspect_socketcan_interface(
    const std::string& interface_name,
    std::string* detail = nullptr);

// Linux SocketCAN transport for raw classic CAN and CAN-FD frames. Classic
// frames are accepted and normalized into CanFdFrame; writes use CAN-FD.
class SocketCanFdSocket
{
public:
    SocketCanFdSocket() = default;
    ~SocketCanFdSocket();

    SocketCanFdSocket(const SocketCanFdSocket&) = delete;
    SocketCanFdSocket& operator=(const SocketCanFdSocket&) = delete;

    bool open_and_bind(
        const std::string& interface_name,
        int receive_timeout_ms,
        std::string* error = nullptr);
    void close() noexcept;
    bool is_open() const noexcept;

    bool send(const CanFdFrame& frame, std::string* error = nullptr) const;
    bool receive(CanFdFrame& frame, std::string* error = nullptr) const;
    bool try_receive(CanFdFrame& frame, std::string* error = nullptr) const;

private:
    std::atomic<int> socket_{-1};
};

}  // namespace sharpa::mujoco::plugins::common
