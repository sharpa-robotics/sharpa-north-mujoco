// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0
#include "sharpa_mujoco/plugins/tactile/tactile_plugin.h"

#include "sharpa_mujoco/plugins/common/udp_transport.h"
#include "sharpa_mujoco/plugins/tactile/tactile_protocol.h"
#include "sharpa_mujoco/runtime.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#include <mujoco/mujoco.h>
#include <msgpack11.hpp>

namespace sharpa::mujoco::plugins::tactile
{
namespace
{

constexpr std::size_t kFingerCount = 10;
constexpr std::size_t kConfigHeaderSize = 12;
constexpr std::size_t kConfigTailSize = 4;

struct FingerDefinition
{
    const char* surface_geom;
    const char* sensor_site;
    std::uint16_t channel;
};

// Platform/tactile SDK channel order: right little..thumb, then left little..thumb.
constexpr std::array<FingerDefinition, kFingerCount> kFingers{{
    {"right_pinky_tactile_surface", "right_pinky_tactile_frame", 0},
    {"right_ring_tactile_surface", "right_ring_tactile_frame", 1},
    {"right_middle_tactile_surface", "right_middle_tactile_frame", 2},
    {"right_index_tactile_surface", "right_index_tactile_frame", 3},
    {"right_thumb_tactile_surface", "right_thumb_tactile_frame", 4},
    {"left_pinky_tactile_surface", "left_pinky_tactile_frame", 5},
    {"left_ring_tactile_surface", "left_ring_tactile_frame", 6},
    {"left_middle_tactile_surface", "left_middle_tactile_frame", 7},
    {"left_index_tactile_surface", "left_index_tactile_frame", 8},
    {"left_thumb_tactile_surface", "left_thumb_tactile_frame", 9},
}};

bool fail(const std::string& message, std::string* error)
{
    if (error) *error = message;
    return false;
}

double unix_timestamp()
{
    return std::chrono::duration<double>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

void rotate_transpose(
    const mjtNum* matrix,
    const mjtNum world[3],
    std::array<float, 3>& local)
{
    for (int row = 0; row < 3; ++row)
    {
        local[static_cast<std::size_t>(row)] = static_cast<float>(
            matrix[row] * world[0] + matrix[3 + row] * world[1] +
            matrix[6 + row] * world[2]);
    }
}

bool send_all(int socket, const std::uint8_t* data, std::size_t size)
{
    std::size_t sent = 0;
    while (sent < size)
    {
        const ssize_t result =
            ::send(socket, data + sent, size - sent, MSG_NOSIGNAL);
        if (result > 0)
        {
            sent += static_cast<std::size_t>(result);
            continue;
        }
        if (result < 0 && errno == EINTR) continue;
        return false;
    }
    return true;
}

bool receive_all(int socket, std::uint8_t* data, std::size_t size)
{
    std::size_t received = 0;
    while (received < size)
    {
        const ssize_t result = ::recv(socket, data + received, size - received, 0);
        if (result > 0)
        {
            received += static_cast<std::size_t>(result);
            continue;
        }
        if (result < 0 && errno == EINTR) continue;
        return false;
    }
    return true;
}

std::uint32_t read_u32_host(const std::uint8_t* data)
{
    std::uint32_t value = 0;
    std::memcpy(&value, data, sizeof(value));
    return value;
}

void write_u32_host(std::uint8_t* data, std::uint32_t value)
{
    std::memcpy(data, &value, sizeof(value));
}

struct ConfigValue
{
    enum class Type { String, Int32, Bool };
    Type type = Type::String;
    std::string string_value;
    std::int32_t int_value = 0;
    bool bool_value = false;
};

struct ConfigItem
{
    std::int32_t channel = 0;
    ConfigValue value;
};

struct ConfigResponse
{
    int code = 0;
    std::string message;
    bool scalar_content = false;
    std::vector<ConfigItem> items;
};

ConfigResponse response(
    int code,
    const std::string& message,
    bool scalar_content = false)
{
    return {code, message, scalar_content, {}};
}

void append_string(std::vector<std::uint8_t>& output, const std::string& value)
{
    if (value.size() <= 31)
        output.push_back(static_cast<std::uint8_t>(0xA0U | value.size()));
    else
    {
        output.push_back(0xD9);
        output.push_back(static_cast<std::uint8_t>(value.size()));
    }
    output.insert(output.end(), value.begin(), value.end());
}

void append_int32(std::vector<std::uint8_t>& output, std::int32_t value)
{
    output.push_back(0xD2);
    const auto bits = static_cast<std::uint32_t>(value);
    output.push_back(static_cast<std::uint8_t>(bits >> 24U));
    output.push_back(static_cast<std::uint8_t>(bits >> 16U));
    output.push_back(static_cast<std::uint8_t>(bits >> 8U));
    output.push_back(static_cast<std::uint8_t>(bits));
}

std::vector<std::uint8_t> encode_response(const ConfigResponse& response_value)
{
    std::vector<std::uint8_t> output;
    output.push_back(0x83);
    append_string(output, "ret_code");
    append_int32(output, response_value.code);
    append_string(output, "msg");
    append_string(output, response_value.message);
    append_string(output, "content");
    if (response_value.scalar_content)
    {
        output.push_back(0xC3);
        return output;
    }
    if (response_value.items.empty())
    {
        output.push_back(0xC0);
        return output;
    }
    output.push_back(static_cast<std::uint8_t>(0x90U |
                                              response_value.items.size()));
    for (const ConfigItem& item : response_value.items)
    {
        output.push_back(0x82);
        append_string(output, "channel");
        append_int32(output, item.channel);
        append_string(output, "value");
        if (item.value.type == ConfigValue::Type::String)
            append_string(output, item.value.string_value);
        else if (item.value.type == ConfigValue::Type::Int32)
            append_int32(output, item.value.int_value);
        else
            output.push_back(item.value.bool_value ? 0xC3 : 0xC2);
    }
    return output;
}

}  // namespace

class TactilePlugin::Impl
{
public:
    explicit Impl(TactilePluginConfig config) : config_(std::move(config)) {}
    ~Impl() { stop(); }

    bool start(Runtime& runtime, std::string* error)
    {
        if (alive_.load()) return true;
        if (!runtime.is_loaded())
            return fail("Tactile plugin requires a loaded model.", error);
        if (config_.name.empty() || config_.sdk_ip.empty() ||
            config_.right_device_ip.empty() || config_.left_device_ip.empty() ||
            config_.right_device_ip == config_.left_device_ip ||
            config_.config_port == 0 ||
            config_.right_destination_port == 0 ||
            config_.left_destination_port == 0 ||
            !std::isfinite(config_.rate_hz) || config_.rate_hz <= 0.0)
            return fail("Tactile plugin configuration is invalid.", error);

        if (!resolve_model(runtime, error))
            return false;
        fps_.store(std::max(1, static_cast<int>(std::lround(config_.rate_hz))));
        if (!tx_socket_.open_and_bind({"0.0.0.0", 0}, 0, error,
                                      common::UdpBindMode::Exclusive))
            return false;
        if (!open_config_servers(error))
        {
            tx_socket_.close();
            return false;
        }
        runtime_ = &runtime;
        alive_.store(true);
        try
        {
            thread_ = std::thread([this] { run(); });
            config_thread_ = std::thread([this] { config_loop(); });
        }
        catch (const std::system_error& exception)
        {
            stop();
            return fail("Tactile worker creation failed: " +
                            std::string(exception.what()),
                        error);
        }
        return true;
    }

    void stop() noexcept
    {
        alive_.store(false);
        wait_cv_.notify_all();
        close_config_servers();
        tx_socket_.close();
        if (thread_.joinable()) thread_.join();
        if (config_thread_.joinable()) config_thread_.join();
        runtime_ = nullptr;
    }

    std::string name() const { return config_.name; }
    bool running() const noexcept { return alive_.load(); }

private:
    bool resolve_model(Runtime& runtime, std::string* error)
    {
        bool valid = true;
        runtime.read_native([&](const mjModel* model, const mjData*) {
            for (std::size_t index = 0; index < kFingers.size(); ++index)
            {
                geom_ids_[index] =
                    mj_name2id(model, mjOBJ_GEOM, kFingers[index].surface_geom);
                // Precise assets retain the native elastomer collision surface.
                if (geom_ids_[index] < 0)
                {
                    std::string precise = kFingers[index].surface_geom;
                    precise.replace(precise.find("_tactile_surface"),
                                    std::string::npos, "_elastomer_collision_01");
                    geom_ids_[index] = mj_name2id(model, mjOBJ_GEOM, precise.c_str());
                }
                site_ids_[index] =
                    mj_name2id(model, mjOBJ_SITE, kFingers[index].sensor_site);
                if (geom_ids_[index] < 0 || site_ids_[index] < 0) valid = false;
            }
        });
        if (!valid)
            return fail(
                "Tactile plugin requires ten named surface geoms and sensor sites.",
                error);
        return true;
    }

    bool open_config_servers(std::string* error)
    {
        const std::array<std::string, 2> ips{{
            config_.right_device_ip, config_.left_device_ip}};
        for (std::size_t index = 0; index < ips.size(); ++index)
        {
            const int socket = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
            if (socket < 0)
            {
                close_config_servers();
                return fail("Tactile config socket() failed.", error);
            }
            int reuse = 1;
            ::setsockopt(socket, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
            sockaddr_in address{};
            address.sin_family = AF_INET;
            address.sin_port = htons(config_.config_port);
            if (::inet_pton(AF_INET, ips[index].c_str(), &address.sin_addr) != 1 ||
                ::bind(socket, reinterpret_cast<const sockaddr*>(&address),
                       sizeof(address)) != 0 ||
                ::listen(socket, 4) != 0)
            {
                ::close(socket);
                close_config_servers();
                return fail("Could not bind tactile config server " + ips[index] +
                                ":" + std::to_string(config_.config_port) + ".",
                            error);
            }
            config_sockets_[index] = socket;
        }
        return true;
    }

    void close_config_servers() noexcept
    {
        for (int& socket : config_sockets_)
        {
            if (socket >= 0)
            {
                ::shutdown(socket, SHUT_RDWR);
                ::close(socket);
                socket = -1;
            }
        }
    }

    void config_loop()
    {
        while (alive_.load())
        {
            std::array<pollfd, 2> polls{{
                {config_sockets_[0], POLLIN, 0},
                {config_sockets_[1], POLLIN, 0},
            }};
            if (::poll(polls.data(), polls.size(), 100) <= 0) continue;
            for (std::size_t index = 0; index < polls.size(); ++index)
            {
                if ((polls[index].revents & POLLIN) == 0) continue;
                const int client = ::accept(config_sockets_[index], nullptr, nullptr);
                if (client < 0) continue;
                timeval timeout{};
                timeout.tv_sec = 1;
                ::setsockopt(client, SOL_SOCKET, SO_RCVTIMEO, &timeout,
                             sizeof(timeout));
                ::setsockopt(client, SOL_SOCKET, SO_SNDTIMEO, &timeout,
                             sizeof(timeout));
                handle_config_client(client, index == 0 ? 0 : 5);
                ::shutdown(client, SHUT_RDWR);
                ::close(client);
            }
        }
    }

    void handle_config_client(int client, int first_channel)
    {
        std::array<std::uint8_t, 4096> buffer{};
        if (!receive_all(client, buffer.data(), kConfigHeaderSize) ||
            buffer[0] != 0xBB || buffer[1] != 0xEE)
            return;
        const std::uint32_t body_size = read_u32_host(buffer.data() + 8);
        if (body_size > buffer.size() - kConfigHeaderSize - kConfigTailSize ||
            !receive_all(client, buffer.data() + kConfigHeaderSize,
                         body_size + kConfigTailSize))
            return;
        const auto tail = buffer.begin() +
            static_cast<std::ptrdiff_t>(kConfigHeaderSize + body_size);
        if (!std::all_of(tail, tail + kConfigTailSize,
                         [](std::uint8_t value) { return value == 0xFF; }))
            return;
        std::string parse_error;
        const msgpack11::MsgPack request = msgpack11::MsgPack::parse(
            reinterpret_cast<const char*>(buffer.data() + kConfigHeaderSize),
            body_size, parse_error);
        if (!parse_error.empty()) return;
        send_config_response(client, handle_config_request(request, first_channel));
    }

    ConfigResponse handle_config_request(
        const msgpack11::MsgPack& request,
        int first_channel)
    {
        const std::string type = request["type"].string_value();
        const std::string key = request["key"].string_value();
        const int requested_channel = request["channel"].int_value();
        if (type == "action")
            return key == "refresh"
                ? response(0, "ok", true)
                : response(1, "unsupported action");
        if (type != "get" && type != "set")
            return response(1, "invalid request type");

        auto channel_selected = [&](int channel) {
            return requested_channel == -1 || requested_channel == channel;
        };
        if (type == "set")
        {
            if (key == "fps")
            {
                const int value = request["value"].int_value();
                if (value <= 0) return response(1, "fps must be positive");
                fps_.store(value);
                wait_cv_.notify_all();
            }
            else if (key == "require_raw")
                require_raw_ = request["value"].bool_value();
            else if (key == "require_infer")
                require_infer_ = request["value"].bool_value();
            else if (key == "require_jpg")
                require_jpg_ = request["value"].bool_value();
            else if (key == "distributed_force")
                distributed_force_ = request["value"].string_value();
            else
                return response(1, "unsupported key");
            return response(0, "ok", true);
        }

        ConfigResponse result = response(0, "ok");
        for (int channel = first_channel; channel < first_channel + 5; ++channel)
        {
            if (!channel_selected(channel)) continue;
            ConfigValue value;
            if (key == "version") value.string_value = "1.3.4";
            else if (key == "wave_tag") value.string_value = "Wave01-B1";
            else if (key == "elastomer_version")
                value.string_value =
                    channel % 5 == 4 ? "M8900001800" : "M8900001400";
            else if (key == "fps")
            {
                value.type = ConfigValue::Type::Int32;
                value.int_value = fps_.load();
            }
            else if (key == "require_raw")
            {
                value.type = ConfigValue::Type::Bool;
                value.bool_value = require_raw_;
            }
            else if (key == "require_infer")
            {
                value.type = ConfigValue::Type::Bool;
                value.bool_value = require_infer_;
            }
            else if (key == "require_jpg")
            {
                value.type = ConfigValue::Type::Bool;
                value.bool_value = require_jpg_;
            }
            else if (key == "distributed_force")
                value.string_value = distributed_force_;
            else return response(1, "unsupported key");
            result.items.push_back(
                {static_cast<std::int32_t>(channel), std::move(value)});
        }
        if (result.items.empty()) return response(1, "channel not found");
        return result;
    }

    void send_config_response(int client, const ConfigResponse& value)
    {
        const std::vector<std::uint8_t> body = encode_response(value);
        std::vector<std::uint8_t> packet(kConfigHeaderSize + body.size() +
                                         kConfigTailSize, 0);
        packet[0] = 0xBB;
        packet[1] = 0xEE;
        write_u32_host(packet.data() + 8,
                       static_cast<std::uint32_t>(body.size()));
        std::memcpy(packet.data() + kConfigHeaderSize, body.data(), body.size());
        std::fill(packet.end() - kConfigTailSize, packet.end(), 0xFF);
        send_all(client, packet.data(), packet.size());
    }

    void run()
    {
        using Clock = std::chrono::steady_clock;
        while (alive_.load())
        {
            const auto cycle_start = Clock::now();
            sample_and_send();
            const int cycle_fps = fps_.load();
            const auto period = std::chrono::duration_cast<Clock::duration>(
                std::chrono::duration<double>(1.0 / cycle_fps));
            std::unique_lock<std::mutex> lock(wait_mutex_);
            wait_cv_.wait_until(lock, cycle_start + period, [this, cycle_fps] {
                return !alive_.load() || fps_.load() != cycle_fps;
            });
        }
    }

    void sample_and_send()
    {
        Runtime* runtime = runtime_;
        if (!runtime || !alive_.load()) return;
        std::array<std::array<float, kF6ValueCount>, kFingerCount> values{};
        if (!runtime->read_native(
                [&](const mjModel* model, const mjData* data) {
                    accumulate_contacts(model, data, values);
                }))
            return;

        const double timestamp = unix_timestamp();
        for (std::size_t index = 0; index < kFingers.size(); ++index)
        {
            F6Frame frame;
            frame.channel = kFingers[index].channel;
            frame.frame_id = frame_ids_[index]++;
            frame.timestamp = timestamp;
            frame.values = values[index];
            std::array<std::uint8_t, kF6PacketSize> encoded{};
            if (!encode_f6_frame(frame, encoded)) continue;
            const std::uint16_t port = frame.channel < 5
                ? config_.right_destination_port
                : config_.left_destination_port;
            tx_socket_.send_to({config_.sdk_ip, port}, encoded.data(),
                               encoded.size());
        }
    }

    void accumulate_contacts(
        const mjModel* model,
        const mjData* data,
        std::array<std::array<float, kF6ValueCount>, kFingerCount>& output) const
    {
        for (int contact_index = 0; contact_index < data->ncon; ++contact_index)
        {
            const mjContact& contact = data->contact[contact_index];
            for (std::size_t finger = 0; finger < kFingerCount; ++finger)
            {
                const bool is_geom1 = contact.geom1 == geom_ids_[finger];
                const bool is_geom2 = contact.geom2 == geom_ids_[finger];
                if (!is_geom1 && !is_geom2) continue;

                mjtNum contact_wrench[6]{};
                mj_contactForce(model, data, contact_index, contact_wrench);
                mjtNum force_world[3]{};
                mjtNum torque_world[3]{};
                // Contact-frame rows are axes in world coordinates. The normal
                // points from geom1 to geom2 and mj_contactForce is the wrench
                // on geom2; reverse it when the tactile surface is geom1.
                const mjtNum sign = is_geom2 ? 1.0 : -1.0;
                for (int axis = 0; axis < 3; ++axis)
                {
                    force_world[axis] = sign * (
                        contact.frame[axis] * contact_wrench[0] +
                        contact.frame[3 + axis] * contact_wrench[1] +
                        contact.frame[6 + axis] * contact_wrench[2]);
                    torque_world[axis] = sign * (
                        contact.frame[axis] * contact_wrench[3] +
                        contact.frame[3 + axis] * contact_wrench[4] +
                        contact.frame[6 + axis] * contact_wrench[5]);
                }

                const int site_id = site_ids_[finger];
                const mjtNum* site_position = data->site_xpos + 3 * site_id;
                const mjtNum* site_rotation = data->site_xmat + 9 * site_id;
                mjtNum lever[3]{
                    contact.pos[0] - site_position[0],
                    contact.pos[1] - site_position[1],
                    contact.pos[2] - site_position[2],
                };
                torque_world[0] +=
                    lever[1] * force_world[2] - lever[2] * force_world[1];
                torque_world[1] +=
                    lever[2] * force_world[0] - lever[0] * force_world[2];
                torque_world[2] +=
                    lever[0] * force_world[1] - lever[1] * force_world[0];
                std::array<float, 3> force_local{};
                std::array<float, 3> torque_local{};
                rotate_transpose(site_rotation, force_world, force_local);
                rotate_transpose(site_rotation, torque_world, torque_local);
                for (std::size_t axis = 0; axis < 3; ++axis)
                {
                    output[finger][axis] += force_local[axis];
                    output[finger][axis + 3] += torque_local[axis];
                }
            }
        }
    }

    TactilePluginConfig config_;
    Runtime* runtime_ = nullptr;
    std::atomic<bool> alive_{false};
    common::UdpSocket tx_socket_;
    std::array<int, kFingerCount> geom_ids_{};
    std::array<int, kFingerCount> site_ids_{};
    std::array<std::uint16_t, kFingerCount> frame_ids_{};
    std::thread thread_;
    std::thread config_thread_;
    std::array<int, 2> config_sockets_{{-1, -1}};
    std::atomic<int> fps_{30};
    bool require_raw_ = false;
    bool require_infer_ = true;
    bool require_jpg_ = true;
    std::string distributed_force_ = "0";
    std::mutex wait_mutex_;
    std::condition_variable wait_cv_;
};

TactilePlugin::TactilePlugin(TactilePluginConfig config)
    : impl_(std::make_unique<Impl>(std::move(config)))
{
}

TactilePlugin::~TactilePlugin() = default;

std::string TactilePlugin::name() const { return impl_->name(); }
bool TactilePlugin::start(Runtime& runtime, std::string* error)
{
    return impl_->start(runtime, error);
}
void TactilePlugin::stop() noexcept { impl_->stop(); }
bool TactilePlugin::is_running() const noexcept { return impl_->running(); }

}  // namespace sharpa::mujoco::plugins::tactile
