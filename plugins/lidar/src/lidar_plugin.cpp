// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#include "sharpa_mujoco/plugins/lidar/lidar_plugin.h"

#include "sharpa_mujoco/plugins/common/mcu_udp_codec.h"
#include "sharpa_mujoco/plugins/common/udp_transport.h"
#include "sharpa_mujoco/plugins/lidar/lidar_protocol.h"
#include "sharpa_mujoco/plugins/lidar/scan_pattern.h"
#include "sharpa_mujoco/runtime.h"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <iostream>
#include <iterator>
#include <limits>
#include <mutex>
#include <numeric>
#include <optional>
#include <system_error>
#include <thread>
#include <utility>

namespace sharpa::mujoco::plugins::lidar
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

bool fail(const std::string& message, std::string* error)
{
    if (error)
    {
        *error = message;
    }
    return false;
}

std::uint64_t realtime_ns()
{
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count());
}

std::array<mjtNum, 3> transform_direction(
    const mjtNum* rotation,
    const std::array<double, 3>& local)
{
    std::array<mjtNum, 3> output{};
    for (int row = 0; row < 3; ++row)
    {
        output[static_cast<std::size_t>(row)] =
            rotation[row * 3] * local[0] +
            rotation[row * 3 + 1] * local[1] +
            rotation[row * 3 + 2] * local[2];
    }
    return output;
}

bool belongs_to_body_subtree(const mjModel* model, int body_id, int root_body_id)
{
    for (int current = body_id; current >= 0; current = model->body_parentid[current])
    {
        if (current == root_body_id)
        {
            return true;
        }
        if (current == 0)
        {
            break;
        }
    }
    return false;
}

bool same_raycast_geometry(const mjModel* model, int first, int second)
{
    if (model->geom_bodyid[first] != model->geom_bodyid[second] ||
        model->geom_type[first] != model->geom_type[second] ||
        model->geom_dataid[first] != model->geom_dataid[second])
    {
        return false;
    }
    for (int axis = 0; axis < 3; ++axis)
    {
        if (model->geom_pos[first * 3 + axis] !=
                model->geom_pos[second * 3 + axis] ||
            model->geom_size[first * 3 + axis] !=
                model->geom_size[second * 3 + axis])
        {
            return false;
        }
    }
    for (int axis = 0; axis < 4; ++axis)
    {
        if (model->geom_quat[first * 4 + axis] !=
            model->geom_quat[second * 4 + axis])
        {
            return false;
        }
    }
    return true;
}

void hide_duplicate_visual_geoms(mjModel* model)
{
    for (int geom_id = 0; geom_id < model->ngeom; ++geom_id)
    {
        if (model->geom_group[geom_id] != 1)
        {
            continue;
        }
        for (int candidate = 0; candidate < model->ngeom; ++candidate)
        {
            if (model->geom_group[candidate] == 1 ||
                model->geom_rgba[candidate * 4 + 3] == 0.0F ||
                !same_raycast_geometry(model, geom_id, candidate))
            {
                continue;
            }
            // The URDF contains coincident display and collision copies of
            // many meshes. Keep one identical surface in the private raycast
            // model so every hit is preserved without tracing it twice.
            model->geom_rgba[geom_id * 4 + 3] = 0.0F;
            break;
        }
    }
}

}  // namespace

class LidarPlugin::Impl
{
public:
    explicit Impl(LidarPluginConfig config)
        : config_(std::move(config))
    {
    }

    ~Impl()
    {
        stop();
    }

    bool start(Runtime& runtime, std::string* error)
    {
        if (running_.load())
        {
            return true;
        }
        if (!runtime.is_loaded() || config_.instances.empty())
        {
            return fail("LiDAR plugin requires a loaded model and at least one instance.", error);
        }
        instances_.clear();
        for (const LidarInstanceConfig& config : config_.instances)
        {
            if (config.name.empty() || config.site_name.empty() ||
                config.bind_port == 0 || config.peer_port == 0 ||
                !std::isfinite(config.scan_rate_hz) ||
                !std::isfinite(config.min_range_m) ||
                !std::isfinite(config.max_range_m) ||
                config.scan_rate_hz <= 0.0 || config.min_range_m < 0.0 ||
                config.max_range_m <= config.min_range_m)
            {
                return fail("LiDAR instance configuration is invalid: " + config.name, error);
            }
            instances_.emplace_back();
            Instance& instance = instances_.back();
            instance.config = config;
            try
            {
                if (!load_scan_pattern(instance, error))
                {
                    instances_.pop_back();
                    stop();
                    return false;
                }
                prepare_directions(instance);
            }
            catch (const std::exception& exception)
            {
                instances_.pop_back();
                stop();
                return fail(
                    "Invalid LiDAR calibration for '" + config.name + "': " +
                        exception.what(),
                    error);
            }
            if (!instance.socket.open_and_bind(
                    {config.bind_ip, config.bind_port}, 0, error))
            {
                instances_.pop_back();
                stop();
                return false;
            }
        }

        if (!runtime.read_native(
                [&](const mjModel* model, const mjData*) {
                    source_model_ = model;
                    model_ = mj_copyModel(nullptr, model);
                    if (!model_)
                    {
                        return;
                    }
                    chassis_root_id_ =
                        mj_name2id(model_, mjOBJ_BODY, "chassis_root");
                    if (chassis_root_id_ < 0)
                    {
                        chassis_root_id_ =
                            mj_name2id(model_, mjOBJ_BODY, "north");
                    }
                    hide_duplicate_visual_geoms(model_);
                    snapshot_ = mj_makeData(model_);
                },
                error))
        {
            stop();
            return fail("LiDAR plugin could not allocate raycast snapshots.", error);
        }
        if (!model_ || !snapshot_)
        {
            stop();
            return fail("LiDAR plugin could not allocate a raycast snapshot.", error);
        }
        if (!configure_raycast_geom_groups(error))
        {
            stop();
            return false;
        }
        for (Instance& instance : instances_)
        {
            const int site_id = mj_name2id(
                model_, mjOBJ_SITE, instance.config.site_name.c_str());
            if (site_id < 0)
            {
                const std::string site_name = instance.config.site_name;
                stop();
                return fail("Unknown LiDAR site: " + site_name, error);
            }
            instance.site_id = site_id;
        }
        const unsigned hardware_threads = std::thread::hardware_concurrency();
        const std::size_t worker_budget = std::max(
            instances_.size(),
            std::min<std::size_t>(
                16,
                hardware_threads > 2 ? hardware_threads - 2 : 1));
        const std::size_t total_rays = std::accumulate(
            instances_.begin(),
            instances_.end(),
            std::size_t{0},
            [](std::size_t sum, const Instance& instance) {
                return sum + instance.directions.size();
            });
        std::size_t workers_left = worker_budget;
        for (std::size_t index = 0; index < instances_.size(); ++index)
        {
            Instance& instance = instances_[index];
            const std::size_t instances_left = instances_.size() - index;
            const std::size_t proportional = static_cast<std::size_t>(std::lround(
                static_cast<double>(worker_budget) *
                static_cast<double>(instance.directions.size()) /
                static_cast<double>(total_rays)));
            const std::size_t automatic_maximum =
                workers_left > instances_left - 1
                ? workers_left - (instances_left - 1)
                : std::size_t{1};
            const std::size_t shard_count =
                instance.config.raycast_threads > 0
                ? std::clamp(
                      instance.config.raycast_threads,
                      std::size_t{1},
                      instance.directions.size())
                : std::clamp(
                      proportional,
                      std::size_t{1},
                      automatic_maximum);
            instance.raycast_data.reserve(shard_count);
            for (std::size_t shard = 0; shard < shard_count; ++shard)
            {
                mjData* data = mj_makeData(model_);
                if (!data)
                {
                    const std::string instance_name = instance.config.name;
                    stop();
                    return fail(
                        "LiDAR plugin could not allocate raycast shards for: " +
                            instance_name,
                        error);
                }
                instance.raycast_data.push_back(data);
            }
            workers_left = shard_count < workers_left
                ? workers_left - shard_count
                : std::size_t{1};
        }
        runtime_ = &runtime;
        running_.store(true);
        try
        {
            for (Instance& instance : instances_)
            {
                instance.raycast_workers.reserve(
                    instance.raycast_data.size() - 1);
                for (std::size_t shard = 1;
                     shard < instance.raycast_data.size();
                     ++shard)
                {
                    instance.raycast_workers.emplace_back(
                        [this, &instance, shard]() {
                            raycast_worker_loop(instance, shard);
                        });
                }
            }
            scheduler_ = std::thread([this]() { scheduler_loop(); });
            for (Instance& instance : instances_)
            {
                instance.sender = std::thread(
                    [this, &instance]() { sender_loop(instance); });
            }
        }
        catch (const std::system_error& exception)
        {
            stop();
            return fail(
                std::string("LiDAR plugin could not create worker threads: ") +
                    exception.what(),
                error);
        }
        return true;
    }

    void stop() noexcept
    {
        running_.store(false);
        scheduler_wakeup_.notify_all();
        if (scheduler_.joinable())
        {
            scheduler_.join();
        }
        for (Instance& instance : instances_)
        {
            {
                std::lock_guard<std::mutex> lock(instance.raycast_mutex);
                instance.stop_raycast_workers = true;
                instance.raycast_job = {};
            }
            instance.raycast_work_ready.notify_all();
            for (std::thread& worker : instance.raycast_workers)
            {
                if (worker.joinable())
                {
                    worker.join();
                }
            }
            instance.raycast_workers.clear();
            {
                std::lock_guard<std::mutex> lock(instance.sender_mutex);
                instance.stop_sender = true;
            }
            instance.sender_ready.notify_all();
            if (instance.sender.joinable())
            {
                instance.sender.join();
            }
            instance.socket.close();
            instance.active_frame.reset();
            instance.pending_frame.reset();
            for (mjData* data : instance.raycast_data)
            {
                mj_deleteData(data);
            }
            instance.raycast_data.clear();
        }
        if (snapshot_)
        {
            mj_deleteData(snapshot_);
            snapshot_ = nullptr;
        }
        if (model_)
        {
            mj_deleteModel(model_);
            model_ = nullptr;
        }
        source_model_ = nullptr;
        chassis_root_id_ = -1;
        instances_.clear();
        runtime_ = nullptr;
    }

    bool is_running() const noexcept
    {
        return running_.load();
    }

private:
    struct Instance
    {
        struct PendingDatagram
        {
            std::chrono::steady_clock::time_point deadline;
            std::vector<std::uint8_t> bytes;
        };

        using PendingFrame = std::vector<PendingDatagram>;

        LidarInstanceConfig config;
        common::UdpSocket socket;
        std::uint32_t packet_sequence = 0;
        std::uint8_t frame_id = 0;
        std::uint16_t jt_sequence = 0;
        ScanPattern scan_pattern;
        int site_id = -1;
        std::array<mjtByte, 6> raycast_geom_groups{1, 1, 1, 1, 1, 1};
        std::vector<mjData*> raycast_data;
        std::vector<std::thread> raycast_workers;
        std::mutex raycast_mutex;
        std::condition_variable raycast_work_ready;
        std::condition_variable raycast_work_done;
        std::function<void(std::size_t)> raycast_job;
        std::size_t raycast_generation = 0;
        std::size_t raycast_workers_completed = 0;
        bool stop_raycast_workers = false;
        std::vector<std::array<double, 3>> directions;
        std::thread sender;
        std::mutex sender_mutex;
        std::condition_variable sender_ready;
        // Keep a currently paced frame plus only the newest complete frame.
        // Dropping an old frame as a whole prevents an overloaded receiver
        // from observing a mix of incomplete revolutions.
        std::optional<PendingFrame> active_frame;
        std::optional<PendingFrame> pending_frame;
        std::size_t active_datagram = 0;
        bool stop_sender = false;
        std::chrono::steady_clock::time_point next_scan_deadline{};
    };

    bool configure_raycast_geom_groups(std::string* error)
    {
        constexpr std::size_t kAvailableExclusionGroups = 5;
        std::vector<std::string> exclusion_profiles;

        for (const Instance& instance : instances_)
        {
            for (const std::string& mesh_name :
                 instance.config.raycast_exclude_meshes)
            {
                if (mj_name2id(model_, mjOBJ_MESH, mesh_name.c_str()) < 0)
                {
                    return fail(
                        "Unknown raycast exclusion mesh for '" +
                            instance.config.name + "': " + mesh_name,
                        error);
                }
            }
        }

        for (int geom_id = 0; geom_id < model_->ngeom; ++geom_id)
        {
            model_->geom_group[geom_id] = 0;
            if (model_->geom_type[geom_id] != mjGEOM_MESH)
            {
                continue;
            }
            const char* mesh_name = mj_id2name(
                model_, mjOBJ_MESH, model_->geom_dataid[geom_id]);
            if (!mesh_name)
            {
                continue;
            }

            std::string profile(instances_.size(), '0');
            for (std::size_t index = 0; index < instances_.size(); ++index)
            {
                const auto& excluded =
                    instances_[index].config.raycast_exclude_meshes;
                if (std::find(excluded.begin(), excluded.end(), mesh_name) !=
                    excluded.end())
                {
                    profile[index] = '1';
                }
            }
            if (profile.find('1') == std::string::npos)
            {
                continue;
            }

            auto found = std::find(
                exclusion_profiles.begin(), exclusion_profiles.end(), profile);
            if (found == exclusion_profiles.end())
            {
                if (exclusion_profiles.size() == kAvailableExclusionGroups)
                {
                    return fail(
                        "LiDAR raycast exclusions require more than five "
                        "distinct mesh groups.",
                        error);
                }
                exclusion_profiles.push_back(profile);
                found = std::prev(exclusion_profiles.end());
            }
            model_->geom_group[geom_id] = static_cast<int>(
                std::distance(exclusion_profiles.begin(), found) + 1);
        }

        for (std::size_t group = 0; group < exclusion_profiles.size(); ++group)
        {
            for (std::size_t index = 0; index < instances_.size(); ++index)
            {
                if (exclusion_profiles[group][index] == '1')
                {
                    instances_[index].raycast_geom_groups[group + 1] = 0;
                }
            }
        }
        return true;
    }

    bool load_scan_pattern(Instance& instance, std::string* error)
    {
        if (instance.config.calibration_file.empty())
        {
            return fail(
                "LiDAR calibration_file is required: " + instance.config.name,
                error);
        }
        return instance.config.model == LidarModel::Ftx140
                   ? load_ftx140_scan_pattern(
                         instance.config.calibration_file,
                         instance.scan_pattern,
                         error)
                   : load_jt16_scan_pattern(
                         instance.config.calibration_file,
                         instance.scan_pattern,
                         error);
    }

    void prepare_directions(Instance& instance)
    {
        if (instance.config.model == LidarModel::Ftx140)
        {
            instance.directions = instance.scan_pattern.ftx140_directions;
            return;
        }

        constexpr std::size_t kAzimuthSamples = 360;
        instance.directions.reserve(kAzimuthSamples * kJt16ChannelCount);
        for (std::size_t azimuth_index = 0;
             azimuth_index < kAzimuthSamples;
             ++azimuth_index)
        {
            const double azimuth =
                2.0 * kPi * static_cast<double>(azimuth_index) /
                kAzimuthSamples;
            for (std::size_t channel = 0; channel < kJt16ChannelCount; ++channel)
            {
                instance.directions.push_back(jt16_direction_platform(
                    instance.scan_pattern.jt16_elevations[channel],
                    azimuth +
                        instance.scan_pattern.jt16_azimuth_offsets[channel]));
            }
        }
    }

    std::vector<LidarReturn> rays(
        const mjModel* model,
        Instance& instance,
        int site_id,
        const std::vector<std::array<double, 3>>& local_directions,
        const LidarInstanceConfig& config,
        int body_exclude)
    {
        const mjtNum* site_rotation =
            instance.raycast_data.front()->site_xmat + site_id * 9;
        std::vector<mjtNum> distances(local_directions.size());
        std::vector<int> geom_ids(local_directions.size(), -1);
        std::vector<mjtNum> directions(local_directions.size() * 3);
        for (std::size_t index = 0; index < local_directions.size(); ++index)
        {
            const std::array<mjtNum, 3> direction =
                transform_direction(site_rotation, local_directions[index]);
            std::copy(
                direction.begin(),
                direction.end(),
                directions.begin() + static_cast<std::ptrdiff_t>(index * 3));
        }
        const auto cast_shard = [&](std::size_t shard) {
            const std::size_t shard_count = instance.raycast_data.size();
            const std::size_t begin = local_directions.size() * shard / shard_count;
            const std::size_t end =
                local_directions.size() * (shard + 1) / shard_count;
            mjData* data = instance.raycast_data[shard];
            mj_multiRay(
                model,
                data,
                data->site_xpos + site_id * 3,
                directions.data() + begin * 3,
                instance.raycast_geom_groups.data(),
                1,
                body_exclude,
                geom_ids.data() + begin,
                distances.data() + begin,
                nullptr,
                static_cast<int>(end - begin),
                config.max_range_m);
        };
        if (instance.raycast_workers.empty())
        {
            cast_shard(0);
        }
        else
        {
            {
                std::lock_guard<std::mutex> lock(instance.raycast_mutex);
                instance.raycast_workers_completed = 0;
                instance.raycast_job = cast_shard;
                ++instance.raycast_generation;
            }
            instance.raycast_work_ready.notify_all();
            cast_shard(0);
            std::unique_lock<std::mutex> lock(instance.raycast_mutex);
            instance.raycast_work_done.wait(lock, [&]() {
                return instance.raycast_workers_completed ==
                       instance.raycast_workers.size();
            });
            instance.raycast_job = {};
        }

        std::vector<LidarReturn> results(local_directions.size());
        for (std::size_t index = 0; index < results.size(); ++index)
        {
            if (distances[index] < config.min_range_m ||
                distances[index] > config.max_range_m ||
                (geom_ids[index] >= 0 && chassis_root_id_ >= 0 &&
                 belongs_to_body_subtree(
                     model, model->geom_bodyid[geom_ids[index]], chassis_root_id_)))
            {
                continue;
            }
            results[index] = {static_cast<double>(distances[index]), 128, true};
        }
        return results;
    }

    bool capture_snapshot()
    {
        Runtime* runtime = runtime_;
        if (!runtime || !snapshot_ || !model_ || !source_model_)
        {
            return false;
        }
        bool copied = false;
        if (!runtime->read_native(
            [&](const mjModel* model, const mjData* data) {
                if (model != source_model_)
                {
                    return;
                }
                mj_copyData(snapshot_, model_, data);
                copied = true;
            }))
        {
            return false;
        }
        return copied;
    }

    void raycast_worker_loop(Instance& instance, std::size_t shard)
    {
        std::size_t observed_generation = 0;
        std::unique_lock<std::mutex> lock(instance.raycast_mutex);
        while (true)
        {
            instance.raycast_work_ready.wait(lock, [&]() {
                return instance.stop_raycast_workers ||
                       instance.raycast_generation != observed_generation;
            });
            if (instance.stop_raycast_workers)
            {
                return;
            }

            observed_generation = instance.raycast_generation;
            const std::function<void(std::size_t)> job = instance.raycast_job;
            lock.unlock();
            job(shard);
            lock.lock();
            ++instance.raycast_workers_completed;
            if (instance.raycast_workers_completed ==
                instance.raycast_workers.size())
            {
                instance.raycast_work_done.notify_one();
            }
        }
    }

    void scan_ftx140(Instance& instance)
    {
        using Clock = std::chrono::steady_clock;
        const Clock::time_point frame_started = Clock::now();
        Runtime* runtime = runtime_;
        if (!runtime)
        {
            return;
        }
        const int site_id = instance.site_id;
        if (site_id < 0)
        {
            return;
        }
        const std::uint64_t timestamp = snapshot_timestamp_ns_;
        const std::vector<LidarReturn> results =
            rays(
                model_,
                instance,
                site_id,
                instance.directions,
                instance.config,
                -1);

        const Clock::duration frame_period =
            std::chrono::duration_cast<Clock::duration>(
                std::chrono::duration<double>(
                    1.0 / instance.config.scan_rate_hz));
        const Clock::time_point send_started = Clock::now();
        const Clock::duration send_budget = std::max(
            Clock::duration::zero(),
            frame_started + frame_period - send_started);
        const Clock::duration packet_period =
            send_budget /
            static_cast<Clock::duration::rep>(kFtx140PacketsPerFrame);
        Clock::time_point deadline = send_started;
        Instance::PendingFrame datagrams;
        datagrams.reserve(kFtx140PacketsPerFrame);
        for (std::size_t packet = 0;
             packet < kFtx140PacketsPerFrame && running_.load() &&
             runtime->is_running();
             ++packet)
        {
            Ftx140PacketMetadata metadata;
            metadata.row_id = static_cast<std::uint8_t>(
                packet / kFtx140ColumnBlockCount);
            metadata.column_id = static_cast<std::uint8_t>(
                packet % kFtx140ColumnBlockCount);
            metadata.frame_id = instance.frame_id;
            metadata.sequence = instance.packet_sequence++;
            metadata.timestamp_ns = timestamp;
            std::array<LidarReturn, kFtx140ChannelsPerPacket> packet_returns{};
            for (std::size_t channel = 0;
                 channel < packet_returns.size();
                 ++channel)
            {
                packet_returns[channel] = results[ftx140_frame_index(
                    metadata.row_id,
                    metadata.column_id,
                    channel)];
            }
            std::vector<std::uint8_t> datagram =
                encode_ftx140_packet(packet_returns, metadata);
            deadline += std::chrono::duration_cast<Clock::duration>(packet_period);
            datagrams.push_back({deadline, std::move(datagram)});
        }
        enqueue_frame(instance, std::move(datagrams));
        ++instance.frame_id;
    }

    void scan_jt16(Instance& instance)
    {
        using Clock = std::chrono::steady_clock;
        const Clock::time_point frame_started = Clock::now();
        Runtime* runtime = runtime_;
        if (!runtime)
        {
            return;
        }
        constexpr std::size_t kAzimuthSamples = 360;
        std::array<std::array<LidarReturn, kJt16ChannelCount>, kAzimuthSamples> packets{};
        const int site_id = instance.site_id;
        if (site_id < 0)
        {
            return;
        }
        const std::uint64_t timestamp = snapshot_timestamp_ns_;
        const std::vector<LidarReturn> results =
            rays(
                model_,
                instance,
                site_id,
                instance.directions,
                instance.config,
                model_->site_bodyid[site_id]);
        for (std::size_t azimuth_index = 0; azimuth_index < kAzimuthSamples; ++azimuth_index)
        {
            for (std::size_t channel = 0; channel < kJt16ChannelCount; ++channel)
            {
                packets[azimuth_index][channel] =
                    results[azimuth_index * kJt16ChannelCount + channel];
            }
        }

        const Clock::duration frame_period =
            std::chrono::duration_cast<Clock::duration>(
                std::chrono::duration<double>(
                    1.0 / instance.config.scan_rate_hz));
        const Clock::time_point send_started = Clock::now();
        const Clock::duration send_budget = std::max(
            Clock::duration::zero(),
            frame_started + frame_period - send_started);
        const Clock::duration packet_period =
            send_budget / static_cast<Clock::duration::rep>(kAzimuthSamples);
        Clock::time_point deadline = send_started;
        Instance::PendingFrame datagrams;
        datagrams.reserve(kAzimuthSamples);
        for (std::size_t azimuth_index = 0;
             azimuth_index < kAzimuthSamples && running_.load() && runtime->is_running();
             ++azimuth_index)
        {
            Jt16PacketMetadata metadata;
            metadata.azimuth_centideg =
                static_cast<std::uint16_t>(azimuth_index * 100);
            metadata.sequence = instance.jt_sequence++;
            metadata.timestamp_ns = timestamp;
            const std::vector<std::uint8_t> payload =
                encode_jt16_packet(packets[azimuth_index], metadata);
            common::McuEnvelope envelope;
            envelope.magic = common::kMcuMagicPoc2;
            envelope.source_code = common::make_source_code(
                common::kMcuDomainSensor, common::kMcuClassLidar);
            envelope.sequence = instance.packet_sequence++;
            envelope.timestamp_ns = timestamp;
            envelope.gptp_ns = timestamp;
            envelope.payload = payload;
            std::vector<std::uint8_t> datagram =
                common::encode_mcu_uplink(envelope);
            // Distribute a revolution over its configured scan period, rather
            // than bursting all 360 packets at once.
            deadline += packet_period;
            datagrams.push_back({deadline, std::move(datagram)});
        }
        enqueue_frame(instance, std::move(datagrams));
    }

    void enqueue_frame(Instance& instance, Instance::PendingFrame frame)
    {
        if (frame.empty())
        {
            return;
        }
        std::lock_guard<std::mutex> lock(instance.sender_mutex);
        // latest-wins backpressure: preserve the in-flight frame's packet
        // pacing, replace only a frame that has not begun transmission.
        instance.pending_frame = std::move(frame);
        instance.sender_ready.notify_one();
    }

    void sender_loop(Instance& instance)
    {
        std::unique_lock<std::mutex> lock(instance.sender_mutex);
        while (running_.load())
        {
            instance.sender_ready.wait(lock, [&]() {
                return instance.stop_sender || instance.active_frame.has_value() ||
                       instance.pending_frame.has_value() ||
                       !running_.load();
            });
            if (instance.stop_sender || !running_.load())
            {
                return;
            }
            if (!instance.active_frame)
            {
                instance.active_frame = std::move(instance.pending_frame);
                instance.pending_frame.reset();
                instance.active_datagram = 0;
            }
            if (!instance.active_frame || instance.active_frame->empty())
            {
                instance.active_frame.reset();
                continue;
            }
            const Instance::PendingDatagram& pending =
                (*instance.active_frame)[instance.active_datagram];
            const auto deadline = pending.deadline;
            if (std::chrono::steady_clock::now() < deadline)
            {
                instance.sender_ready.wait_until(lock, deadline, [&]() {
                    return instance.stop_sender || !running_.load();
                });
                continue;
            }
            std::vector<std::uint8_t> datagram =
                std::move((*instance.active_frame)[instance.active_datagram].bytes);
            ++instance.active_datagram;
            if (instance.active_datagram == instance.active_frame->size())
            {
                instance.active_frame.reset();
                instance.active_datagram = 0;
            }
            lock.unlock();
            std::string send_error;
            if (!instance.socket.send_to(
                {instance.config.peer_ip, instance.config.peer_port},
                datagram.data(), datagram.size(), &send_error))
            {
                std::clog << "LiDAR UDP send failed for '" << instance.config.name
                          << "': " << send_error << '\n';
            }
            lock.lock();
        }
    }

    void scheduler_loop()
    {
        using Clock = std::chrono::steady_clock;
        for (Instance& instance : instances_)
        {
            instance.next_scan_deadline = Clock::now();
        }
        while (running_.load())
        {
            Runtime* runtime = runtime_;
            if (!runtime || !runtime->is_running())
            {
                std::unique_lock<std::mutex> lock(scheduler_mutex_);
                scheduler_wakeup_.wait_for(lock, std::chrono::milliseconds(2));
                continue;
            }

            const Clock::time_point now = Clock::now();
            std::vector<Instance*> due;
            Clock::time_point next_deadline = Clock::time_point::max();
            for (Instance& instance : instances_)
            {
                if (instance.next_scan_deadline <= now)
                {
                    due.push_back(&instance);
                }
                next_deadline = std::min(next_deadline, instance.next_scan_deadline);
            }
            if (due.empty())
            {
                std::unique_lock<std::mutex> lock(scheduler_mutex_);
                scheduler_wakeup_.wait_until(lock, next_deadline);
                continue;
            }

            // A scheduler cycle reads MuJoCo exactly once.  Every due sensor
            // subsequently traverses the same immutable state epoch.
            if (!capture_snapshot())
            {
                continue;
            }
            snapshot_timestamp_ns_ = realtime_ns();
            for (Instance* instance : due)
            {
                for (mjData* data : instance->raycast_data)
                {
                    mj_copyData(data, model_, snapshot_);
                }
            }
            const auto process_instance = [this](Instance& instance) {
                if (instance.config.model == LidarModel::Ftx140)
                {
                    scan_ftx140(instance);
                }
                else
                {
                    scan_jt16(instance);
                }
            };

            // MuJoCo supports parallel sampling with one mjData per worker.
            // Each sensor owns a copy of this batch's immutable snapshot.
            std::vector<std::thread> workers;
            workers.reserve(due.size());
            std::size_t launched = 0;
            try
            {
                for (; launched < due.size(); ++launched)
                {
                    workers.emplace_back(process_instance, std::ref(*due[launched]));
                }
            }
            catch (const std::system_error& exception)
            {
                std::clog << "LiDAR scan worker creation failed; completing this "
                          << "batch serially: " << exception.what() << '\n';
            }
            for (std::thread& worker : workers)
            {
                worker.join();
            }
            for (; launched < due.size(); ++launched)
            {
                process_instance(*due[launched]);
            }

            // Do this after every worker has completed so an instance cannot
            // be scheduled into a later snapshot while still reading this one.
            for (Instance* instance : due)
            {
                const Clock::duration period = std::chrono::duration_cast<Clock::duration>(
                    std::chrono::duration<double>(1.0 / instance->config.scan_rate_hz));
                instance->next_scan_deadline += period;
                if (instance->next_scan_deadline < Clock::now())
                {
                    instance->next_scan_deadline = Clock::now();
                }
            }
        }
    }

    LidarPluginConfig config_;
    std::deque<Instance> instances_;
    Runtime* runtime_ = nullptr;
    const mjModel* source_model_ = nullptr;
    mjModel* model_ = nullptr;
    mjData* snapshot_ = nullptr;
    int chassis_root_id_ = -1;
    std::uint64_t snapshot_timestamp_ns_ = 0;
    std::thread scheduler_;
    std::mutex scheduler_mutex_;
    std::condition_variable scheduler_wakeup_;
    std::atomic<bool> running_{false};
};

LidarPlugin::LidarPlugin(LidarPluginConfig config)
    : impl_(std::make_unique<Impl>(std::move(config)))
{
}

LidarPlugin::~LidarPlugin() = default;

std::string LidarPlugin::name() const
{
    return "lidar";
}

bool LidarPlugin::start(Runtime& runtime, std::string* error)
{
    return impl_->start(runtime, error);
}

void LidarPlugin::stop() noexcept
{
    impl_->stop();
}

bool LidarPlugin::is_running() const noexcept
{
    return impl_->is_running();
}

}  // namespace sharpa::mujoco::plugins::lidar
