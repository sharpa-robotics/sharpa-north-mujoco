// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#include "lidar_config.h"
#include "simulator_config.h"

#include "sharpa_mujoco/plugins/common/mcu_udp_codec.h"
#include "sharpa_mujoco/plugins/common/udp_transport.h"
#include "sharpa_mujoco/plugins/lidar/scan_pattern.h"

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace
{

using sharpa::mujoco::plugins::common::UdpEndpoint;
using sharpa::mujoco::plugins::common::UdpSocket;
using sharpa::mujoco::simulator::LidarMount;

constexpr double kPi = 3.14159265358979323846;
constexpr std::size_t kFtxPacketSize = 554;
constexpr std::size_t kFtxChannelOffset = 24;
constexpr std::size_t kFtxTailOffset = 504;
constexpr std::size_t kJtPacketSize = 80;
constexpr std::size_t kMcuEnvelopeSize = 32;
constexpr int kMaxSceneGeoms = 6000;
constexpr double kAxisLength = 0.22;
constexpr double kCenterRayLength = 0.32;

struct Point
{
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
    bool valid = false;
};

struct Cloud
{
    std::mutex mutex;
    std::vector<Point> points;
    std::uint8_t frame_id = 0;
    bool has_frame = false;
    std::size_t last_azimuth_index = 0;
    bool has_azimuth = false;
    bool has_completed_frame = false;
    std::size_t frames_in_window = 0;
    double frame_rate_hz = 0.0;
    std::chrono::steady_clock::time_point rate_window_start =
        std::chrono::steady_clock::now();
};

struct SensorTransform
{
    std::array<mjtNum, 3> position{};
    std::array<mjtNum, 9> rotation{};
};

using WorldPoint = std::array<float, 3>;

struct MouseState
{
    bool left = false;
    bool middle = false;
    bool right = false;
    double last_x = 0.0;
    double last_y = 0.0;
    mjModel* model = nullptr;
    mjvScene* scene = nullptr;
    mjvCamera* camera = nullptr;
    std::array<mjvCamera, 6>* cameras = nullptr;
    std::array<mjvCamera, 6>* default_cameras = nullptr;
    int selected_view = 0;
};

Point from_angles(double distance, double horizontal, double vertical)
{
    const auto direction =
        sharpa::mujoco::plugins::lidar::jt16_direction_platform(
            vertical * 180.0 / kPi, horizontal);
    return {
        static_cast<float>(distance * direction[0]),
        static_cast<float>(distance * direction[1]),
        static_cast<float>(distance * direction[2]),
        true};
}

void record_completed_frame(Cloud& cloud)
{
    const auto now = std::chrono::steady_clock::now();
    ++cloud.frames_in_window;
    cloud.has_completed_frame = true;
    const double elapsed =
        std::chrono::duration<double>(now - cloud.rate_window_start).count();
    if (elapsed >= 1.0)
    {
        cloud.frame_rate_hz =
            static_cast<double>(cloud.frames_in_window) / elapsed;
        cloud.frames_in_window = 0;
        cloud.rate_window_start = now;
    }
}

bool snapshot_ready(Cloud& cloud)
{
    std::lock_guard<std::mutex> lock(cloud.mutex);
    return cloud.has_completed_frame;
}

void receive_ftx(
    const UdpEndpoint& endpoint,
    Cloud& cloud,
    const sharpa::mujoco::plugins::lidar::ScanPattern& pattern,
    std::atomic<bool>& running)
{
    UdpSocket socket;
    std::string error;
    if (!socket.open_and_bind(endpoint, 50, &error))
    {
        std::cerr << error << '\n';
        return;
    }
    constexpr std::size_t kWidth = 256;
    constexpr std::size_t kHeight = 192;
    {
        std::lock_guard<std::mutex> lock(cloud.mutex);
        cloud.points.assign(kWidth * kHeight, Point{});
    }
    std::array<std::uint8_t, 2048> buffer{};
    while (running.load())
    {
        std::size_t received = 0;
        if (!socket.receive(buffer.data(), buffer.size(), received) ||
            received != kFtxPacketSize || buffer[0] != 0xEE ||
            buffer[1] != 0xFF)
        {
            continue;
        }
        const std::uint8_t column_block = buffer[kFtxTailOffset + 7];
        const std::uint8_t row_block = buffer[kFtxTailOffset + 8];
        const std::uint8_t frame_id = buffer[kFtxTailOffset + 9];
        if (column_block >= 16 || row_block >= 32)
        {
            continue;
        }
        std::array<Point, 96> packet_points{};
        for (std::size_t channel = 0; channel < packet_points.size(); ++channel)
        {
            const std::uint16_t raw =
                sharpa::mujoco::plugins::common::read_le16(
                    buffer.data() + kFtxChannelOffset + channel * 5);
            if (raw == 0)
            {
                continue;
            }
            const std::size_t row =
                static_cast<std::size_t>(row_block) * 6 + channel % 6;
            const std::size_t column =
                static_cast<std::size_t>(column_block) * 16 + channel / 6;
            const auto& direction = pattern.ftx140_directions[
                row * sharpa::mujoco::plugins::lidar::kFtx140Columns + column];
            const float distance = static_cast<float>(raw) * 0.004F;
            packet_points[channel] = {
                distance * static_cast<float>(direction[0]),
                distance * static_cast<float>(direction[1]),
                distance * static_cast<float>(direction[2]),
                true};
        }
        std::lock_guard<std::mutex> lock(cloud.mutex);
        if (cloud.has_frame && cloud.frame_id != frame_id)
        {
            record_completed_frame(cloud);
        }
        for (std::size_t channel = 0; channel < packet_points.size(); ++channel)
        {
            const std::size_t row =
                static_cast<std::size_t>(row_block) * 6 + channel % 6;
            const std::size_t column =
                static_cast<std::size_t>(column_block) * 16 + channel / 6;
            cloud.points[row * kWidth + column] = packet_points[channel];
        }
        cloud.frame_id = frame_id;
        cloud.has_frame = true;
    }
}

void receive_jt16(
    const UdpEndpoint& endpoint,
    Cloud& cloud,
    const sharpa::mujoco::plugins::lidar::ScanPattern& pattern,
    std::atomic<bool>& running)
{
    UdpSocket socket;
    std::string error;
    if (!socket.open_and_bind(endpoint, 50, &error))
    {
        std::cerr << error << '\n';
        return;
    }
    constexpr std::size_t kAzimuthSamples = 360;
    {
        std::lock_guard<std::mutex> lock(cloud.mutex);
        cloud.points.assign(
            kAzimuthSamples * pattern.jt16_elevations.size(), Point{});
    }
    std::array<std::uint8_t, 2048> buffer{};
    while (running.load())
    {
        std::size_t received = 0;
        if (!socket.receive(buffer.data(), buffer.size(), received) ||
            received < kMcuEnvelopeSize + kJtPacketSize)
        {
            continue;
        }
        const std::uint8_t* packet = buffer.data() + kMcuEnvelopeSize;
        if (packet[0] != 0xEE || packet[1] != 0xFF)
        {
            continue;
        }
        const std::uint16_t azimuth_centideg =
            sharpa::mujoco::plugins::common::read_le16(packet + 16);
        const double azimuth =
            static_cast<double>(azimuth_centideg) * 0.01 * kPi / 180.0;
        const std::size_t azimuth_index = std::min(
            kAzimuthSamples - 1,
            static_cast<std::size_t>(azimuth_centideg / 100));
        std::lock_guard<std::mutex> lock(cloud.mutex);
        if (cloud.has_azimuth && azimuth_index < cloud.last_azimuth_index)
        {
            record_completed_frame(cloud);
        }
        cloud.last_azimuth_index = azimuth_index;
        cloud.has_azimuth = true;
        for (std::size_t channel = 0;
             channel < pattern.jt16_elevations.size(); ++channel)
        {
            const std::uint16_t raw =
                sharpa::mujoco::plugins::common::read_le16(
                    packet + 18 + channel * 3);
            Point& point = cloud.points[
                azimuth_index * pattern.jt16_elevations.size() + channel];
            point = raw == 0
                        ? Point{}
                        : from_angles(
                              static_cast<double>(raw) * 0.004,
                              azimuth + pattern.jt16_azimuth_offsets[channel],
                              pattern.jt16_elevations[channel] * kPi / 180.0);
        }
    }
}

SensorTransform sensor_transform(
    const mjModel* model,
    const mjData* data,
    const LidarMount& mount)
{
    const int body_id = mj_name2id(
        model, mjOBJ_BODY, mount.parent_body.c_str());
    if (body_id < 0)
    {
        throw std::runtime_error("Unknown LiDAR parent body: " + mount.parent_body);
    }
    const mjtNum* body_rotation = data->xmat + body_id * 9;
    const mjtNum* body_position = data->xpos + body_id * 3;
    std::array<mjtNum, 9> mount_rotation{};
    mju_quat2Mat(mount_rotation.data(), mount.quaternion.data());
    SensorTransform output;
    for (int row = 0; row < 3; ++row)
    {
        output.position[row] = body_position[row];
        for (int inner = 0; inner < 3; ++inner)
        {
            output.position[row] +=
                body_rotation[row * 3 + inner] * mount.position[inner];
        }
        for (int column = 0; column < 3; ++column)
        {
            for (int inner = 0; inner < 3; ++inner)
            {
                output.rotation[row * 3 + column] +=
                    body_rotation[row * 3 + inner] *
                    mount_rotation[inner * 3 + column];
            }
        }
    }
    return output;
}

std::array<mjtNum, 3> transform_point(
    const SensorTransform& transform,
    const Point& point)
{
    return {
        transform.position[0] + transform.rotation[0] * point.x +
            transform.rotation[1] * point.y + transform.rotation[2] * point.z,
        transform.position[1] + transform.rotation[3] * point.x +
            transform.rotation[4] * point.y + transform.rotation[5] * point.z,
        transform.position[2] + transform.rotation[6] * point.x +
            transform.rotation[7] * point.y + transform.rotation[8] * point.z};
}

std::vector<WorldPoint> snapshot_cloud(
    Cloud& cloud,
    const SensorTransform& transform)
{
    std::vector<WorldPoint> output;
    std::lock_guard<std::mutex> lock(cloud.mutex);
    output.reserve(cloud.points.size());
    for (const Point& point : cloud.points)
    {
        if (!point.valid)
        {
            continue;
        }
        const auto world = transform_point(transform, point);
        output.push_back({
            static_cast<float>(world[0]),
            static_cast<float>(world[1]),
            static_cast<float>(world[2])});
    }
    return output;
}

void draw_points(
    const std::vector<WorldPoint>& points,
    const std::array<float, 3>& color)
{
    glColor3f(color[0], color[1], color[2]);
    glBegin(GL_POINTS);
    for (const WorldPoint& point : points)
    {
        glVertex3fv(point.data());
    }
    glEnd();

}

void draw_origin_axes_and_center_ray(
    const SensorTransform& transform,
    const std::array<double, 3>& reference_ray)
{
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);
    glPointSize(20.0F);
    glBegin(GL_POINTS);
    glColor3f(1.0F, 0.95F, 0.05F);
    glVertex3dv(transform.position.data());
    glEnd();

    // Dark halo keeps all three axes readable over bright robot meshes and clouds.
    glLineWidth(12.0F);
    glColor3f(0.02F, 0.02F, 0.02F);
    glBegin(GL_LINES);
    for (int axis = 0; axis < 3; ++axis)
    {
        glVertex3dv(transform.position.data());
        glVertex3d(
            transform.position[0] + kAxisLength * transform.rotation[axis],
            transform.position[1] + kAxisLength * transform.rotation[3 + axis],
            transform.position[2] + kAxisLength * transform.rotation[6 + axis]);
    }
    glEnd();

    glLineWidth(8.0F);
    glBegin(GL_LINES);
    for (int axis = 0; axis < 3; ++axis)
    {
        if (axis == 0) glColor3f(1.0F, 0.0F, 0.0F);
        if (axis == 1) glColor3f(0.0F, 1.0F, 0.0F);
        if (axis == 2) glColor3f(0.0F, 0.35F, 1.0F);
        glVertex3dv(transform.position.data());
        glVertex3d(
            transform.position[0] + kAxisLength * transform.rotation[axis],
            transform.position[1] + kAxisLength * transform.rotation[3 + axis],
            transform.position[2] + kAxisLength * transform.rotation[6 + axis]);
    }
    glEnd();

    glLineWidth(6.0F);
    glColor3f(1.0F, 0.85F, 0.05F);
    glBegin(GL_LINES);
    glVertex3d(
        transform.position[0] + kAxisLength *
            (transform.rotation[0] * reference_ray[0] +
             transform.rotation[1] * reference_ray[1] +
             transform.rotation[2] * reference_ray[2]),
        transform.position[1] + kAxisLength *
            (transform.rotation[3] * reference_ray[0] +
             transform.rotation[4] * reference_ray[1] +
             transform.rotation[5] * reference_ray[2]),
        transform.position[2] + kAxisLength *
            (transform.rotation[6] * reference_ray[0] +
             transform.rotation[7] * reference_ray[1] +
             transform.rotation[8] * reference_ray[2]));
    glVertex3d(
        transform.position[0] + kCenterRayLength *
            (transform.rotation[0] * reference_ray[0] +
             transform.rotation[1] * reference_ray[1] +
             transform.rotation[2] * reference_ray[2]),
        transform.position[1] + kCenterRayLength *
            (transform.rotation[3] * reference_ray[0] +
             transform.rotation[4] * reference_ray[1] +
             transform.rotation[5] * reference_ray[2]),
        transform.position[2] + kCenterRayLength *
            (transform.rotation[6] * reference_ray[0] +
             transform.rotation[7] * reference_ray[1] +
             transform.rotation[8] * reference_ray[2]));
    glEnd();
    glLineWidth(1.0F);
    glEnable(GL_DEPTH_TEST);
}

void render_view(
    const mjModel* model,
    mjData* data,
    mjvOption& option,
    mjvScene& scene,
    mjrContext& context,
    mjvCamera camera,
    const mjrRect& viewport,
    const std::vector<WorldPoint>& cloud,
    const SensorTransform& transform,
    const std::array<float, 3>& color,
    const char* title,
    const char* axes,
    const std::array<double, 3>& reference_ray = {1.0, 0.0, 0.0})
{
    mjv_updateScene(
        model, data, &option, nullptr, &camera, mjCAT_ALL, &scene);
    mjr_render(viewport, &scene, &context);

    glDisable(GL_LIGHTING);
    glPointSize(3.0F);
    draw_points(cloud, color);
    draw_origin_axes_and_center_ray(transform, reference_ray);
    mjr_overlay(mjFONT_BIG, mjGRID_TOPLEFT, viewport, title, axes, &context);
}

mjvCamera cloud_camera(
    const mjvCamera& base,
    const std::vector<WorldPoint>& cloud,
    const SensorTransform& transform)
{
    mjvCamera camera = base;
    std::array<double, 3> minimum{
        transform.position[0], transform.position[1], transform.position[2]};
    std::array<double, 3> maximum = minimum;
    for (const WorldPoint& point : cloud)
    {
        for (int axis = 0; axis < 3; ++axis)
        {
            minimum[axis] = std::min(minimum[axis], static_cast<double>(point[axis]));
            maximum[axis] = std::max(maximum[axis], static_cast<double>(point[axis]));
        }
    }
    double span = 1.0;
    for (int axis = 0; axis < 3; ++axis)
    {
        camera.lookat[axis] = 0.5 * (minimum[axis] + maximum[axis]);
        span = std::max(span, maximum[axis] - minimum[axis]);
    }
    camera.distance = std::clamp(1.15 * span, 1.0, 35.0);
    camera.azimuth = 135.0;
    camera.elevation = -35.0;
    return camera;
}

std::string status(Cloud& cloud)
{
    std::lock_guard<std::mutex> lock(cloud.mutex);
    const std::size_t valid = static_cast<std::size_t>(std::count_if(
        cloud.points.begin(), cloud.points.end(),
        [](const Point& point) { return point.valid; }));
    return std::to_string(valid) + " pts " +
           std::to_string(static_cast<int>(std::lround(cloud.frame_rate_hz))) +
           " Hz";
}

void mouse_button(GLFWwindow* window, int button, int action, int)
{
    auto* state = static_cast<MouseState*>(glfwGetWindowUserPointer(window));
    state->left =
        glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
    state->middle =
        glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
    state->right =
        glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
    if (action == GLFW_PRESS)
    {
        glfwGetCursorPos(window, &state->last_x, &state->last_y);
        int width = 0;
        int height = 0;
        glfwGetWindowSize(window, &width, &height);
        const int column = std::clamp(
            static_cast<int>(3.0 * state->last_x / std::max(1, width)),
            0,
            2);
        const int row = state->last_y < 0.5 * height ? 0 : 1;
        state->selected_view = row * 3 + column;
        state->camera = &(*state->cameras)[state->selected_view];
    }
    (void)button;
}

void mouse_move(GLFWwindow* window, double x, double y)
{
    auto* state = static_cast<MouseState*>(glfwGetWindowUserPointer(window));
    if (!state->left && !state->middle && !state->right)
    {
        return;
    }
    int width = 0;
    int height = 0;
    glfwGetWindowSize(window, &width, &height);
    const double dx = (x - state->last_x) / std::max(1, height);
    const double dy = (y - state->last_y) / std::max(1, height);
    state->last_x = x;
    state->last_y = y;
    const bool shift =
        glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
        glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
    int action = mjMOUSE_ROTATE_V;
    if (state->right)
    {
        action = shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    }
    else if (state->middle)
    {
        action = mjMOUSE_ZOOM;
    }
    else if (shift)
    {
        action = mjMOUSE_ROTATE_H;
    }
    mjv_moveCamera(
        state->model, action, dx, dy, state->scene, state->camera);
}

void keyboard(GLFWwindow* window, int key, int, int action, int)
{
    if (action != GLFW_PRESS && action != GLFW_REPEAT)
    {
        return;
    }
    auto* state = static_cast<MouseState*>(glfwGetWindowUserPointer(window));
    if (key == GLFW_KEY_R)
    {
        *state->camera = (*state->default_cameras)[state->selected_view];
    }
    else if (key == GLFW_KEY_LEFT || key == GLFW_KEY_RIGHT)
    {
        mjv_moveCamera(
            state->model,
            mjMOUSE_ROTATE_H,
            key == GLFW_KEY_LEFT ? -0.04 : 0.04,
            0.0,
            state->scene,
            state->camera);
    }
    else if (key == GLFW_KEY_UP || key == GLFW_KEY_DOWN)
    {
        mjv_moveCamera(
            state->model,
            mjMOUSE_ROTATE_V,
            0.0,
            key == GLFW_KEY_UP ? -0.04 : 0.04,
            state->scene,
            state->camera);
    }
    else if (key == GLFW_KEY_W || key == GLFW_KEY_S)
    {
        mjv_moveCamera(
            state->model,
            mjMOUSE_MOVE_V,
            0.0,
            key == GLFW_KEY_W ? -0.03 : 0.03,
            state->scene,
            state->camera);
    }
    else if (key == GLFW_KEY_A || key == GLFW_KEY_D)
    {
        mjv_moveCamera(
            state->model,
            mjMOUSE_MOVE_V,
            key == GLFW_KEY_A ? -0.03 : 0.03,
            0.0,
            state->scene,
            state->camera);
    }
    else if (
        key == GLFW_KEY_EQUAL || key == GLFW_KEY_KP_ADD ||
        key == GLFW_KEY_MINUS || key == GLFW_KEY_KP_SUBTRACT)
    {
        const bool zoom_in = key == GLFW_KEY_EQUAL || key == GLFW_KEY_KP_ADD;
        mjv_moveCamera(
            state->model,
            mjMOUSE_ZOOM,
            0.0,
            zoom_in ? -0.08 : 0.08,
            state->scene,
            state->camera);
    }
}

void scroll(GLFWwindow* window, double, double yoffset)
{
    auto* state = static_cast<MouseState*>(glfwGetWindowUserPointer(window));
    mjv_moveCamera(
        state->model,
        mjMOUSE_ZOOM,
        0.0,
        -0.05 * yoffset,
        state->scene,
        state->camera);
}

}  // namespace

int main(int argc, char** argv)
{
    std::string config_path;
    std::string model_override;
    for (int index = 1; index < argc; ++index)
    {
        const std::string argument = argv[index];
        if ((argument == "--config" || argument == "--model") &&
            index + 1 < argc)
        {
            const std::string value = argv[++index];
            if (argument == "--config")
            {
                config_path = value;
            }
            else
            {
                model_override = value;
            }
            continue;
        }
        std::cerr << "Usage: " << argv[0]
                  << " --config <path> [--model <path>]\n";
        return EXIT_FAILURE;
    }
    if (config_path.empty())
    {
        std::cerr << "--config is required\n";
        return EXIT_FAILURE;
    }

    sharpa::mujoco::simulator::SimulatorConfig config =
        sharpa::mujoco::simulator::default_config();
    std::string error;
    if (!sharpa::mujoco::simulator::load_config_file(
            config_path, config, &error))
    {
        std::cerr << error << '\n';
        return EXIT_FAILURE;
    }
    if (!model_override.empty())
    {
        config.model_path = model_override;
    }
    std::vector<LidarMount> mounts;
    if (!sharpa::mujoco::simulator::parse_lidar_mounts(
            config.devices, mounts, &error))
    {
        std::cerr << error << '\n';
        return EXIT_FAILURE;
    }
    const auto find_mount = [&](const std::string& name) {
        return std::find_if(
            mounts.begin(), mounts.end(),
            [&](const LidarMount& mount) { return mount.instance.name == name; });
    };
    const auto front_mount = find_mount("chest_lidar_front");
    const auto rear_mount = find_mount("chest_lidar_rear");
    const auto chassis_mount = find_mount("chassis_lidar_front");
    if (front_mount == mounts.end() || rear_mount == mounts.end() ||
        chassis_mount == mounts.end())
    {
        std::cerr << "All three LiDAR devices must be enabled.\n";
        return EXIT_FAILURE;
    }

    std::array<char, 1024> load_error{};
    mjModel* model = mj_loadXML(
        config.model_path.c_str(), nullptr, load_error.data(),
        static_cast<int>(load_error.size()));
    if (!model)
    {
        std::cerr << load_error.data() << '\n';
        return EXIT_FAILURE;
    }
    mjData* data = mj_makeData(model);
    if (!data)
    {
        std::cerr << "Could not allocate MuJoCo data.\n";
        mj_deleteModel(model);
        return EXIT_FAILURE;
    }
    mj_forward(model, data);
    std::array<SensorTransform, 3> transforms{
        sensor_transform(model, data, *front_mount),
        sensor_transform(model, data, *rear_mount),
        sensor_transform(model, data, *chassis_mount)};

    sharpa::mujoco::plugins::lidar::ScanPattern ftx_pattern;
    sharpa::mujoco::plugins::lidar::ScanPattern jt16_pattern;
    if (!sharpa::mujoco::plugins::lidar::load_ftx140_scan_pattern(
            front_mount->instance.calibration_file, ftx_pattern, &error) ||
        !sharpa::mujoco::plugins::lidar::load_jt16_scan_pattern(
            chassis_mount->instance.calibration_file, jt16_pattern, &error))
    {
        std::cerr << error << '\n';
        mj_deleteData(data);
        mj_deleteModel(model);
        return EXIT_FAILURE;
    }
    const std::array<double, 3> jt16_reference_ray =
        sharpa::mujoco::plugins::lidar::jt16_direction_platform(
            jt16_pattern.jt16_elevations.front(),
            jt16_pattern.jt16_azimuth_offsets.front());

    if (!glfwInit())
    {
        mj_deleteData(data);
        mj_deleteModel(model);
        return EXIT_FAILURE;
    }
    GLFWwindow* window = glfwCreateWindow(
        1800, 1000, "Three LiDAR alignment views", nullptr, nullptr);
    if (!window)
    {
        glfwTerminate();
        mj_deleteData(data);
        mj_deleteModel(model);
        return EXIT_FAILURE;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    mjvCamera camera;
    mjvOption option;
    mjvScene scene;
    mjrContext context;
    mjv_defaultCamera(&camera);
    mjv_defaultOption(&option);
    mjv_defaultScene(&scene);
    mjr_defaultContext(&context);
    mjv_makeScene(model, &scene, kMaxSceneGeoms);
    mjr_makeContext(model, &context, mjFONTSCALE_150);
    camera.orthographic = 1;

    std::array<mjvCamera, 6> cameras;
    cameras.fill(camera);
    std::copy(
        transforms[0].position.begin(), transforms[0].position.end(),
        cameras[0].lookat);
    cameras[0].distance = 0.55;
    cameras[0].azimuth = 155.0;
    cameras[0].elevation = -18.0;
    std::copy(
        transforms[1].position.begin(), transforms[1].position.end(),
        cameras[1].lookat);
    cameras[1].distance = 0.55;
    cameras[1].azimuth = -25.0;
    cameras[1].elevation = -18.0;
    std::copy(
        transforms[2].position.begin(), transforms[2].position.end(),
        cameras[2].lookat);
    cameras[2].distance = 0.65;
    cameras[2].azimuth = 145.0;
    cameras[2].elevation = -25.0;
    std::array<mjvCamera, 6> default_cameras = cameras;
    MouseState mouse_state;
    mouse_state.model = model;
    mouse_state.scene = &scene;
    mouse_state.camera = &cameras[0];
    mouse_state.cameras = &cameras;
    mouse_state.default_cameras = &default_cameras;
    glfwSetWindowUserPointer(window, &mouse_state);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetKeyCallback(window, keyboard);
    glfwSetScrollCallback(window, scroll);

    std::atomic<bool> running{true};
    std::array<Cloud, 3> clouds;
    std::thread front([&]() {
        receive_ftx(
            {front_mount->instance.peer_ip, front_mount->instance.peer_port},
            clouds[0], ftx_pattern, running);
    });
    std::thread rear([&]() {
        receive_ftx(
            {rear_mount->instance.peer_ip, rear_mount->instance.peer_port},
            clouds[1], ftx_pattern, running);
    });
    std::thread chassis([&]() {
        receive_jt16(
            {chassis_mount->instance.peer_ip, chassis_mount->instance.peer_port},
            clouds[2], jt16_pattern, running);
    });
    auto next_title = std::chrono::steady_clock::now();
    auto next_render = std::chrono::steady_clock::now();
    bool snapshot_built = false;
    std::array<std::vector<WorldPoint>, 3> world_clouds;

    while (!glfwWindowShouldClose(window))
    {
        glfwPollEvents();
        const auto now = std::chrono::steady_clock::now();
        if (!snapshot_built &&
            snapshot_ready(clouds[0]) &&
            snapshot_ready(clouds[1]) &&
            snapshot_ready(clouds[2]))
        {
            world_clouds[0] = snapshot_cloud(clouds[0], transforms[0]);
            world_clouds[1] = snapshot_cloud(clouds[1], transforms[1]);
            world_clouds[2] = snapshot_cloud(clouds[2], transforms[2]);
            for (int index = 0; index < 3; ++index)
            {
                cameras[3 + index] = cloud_camera(
                    camera, world_clouds[index], transforms[index]);
                default_cameras[3 + index] = cameras[3 + index];
            }
            snapshot_built = true;
        }

        if (now < next_render)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }
        next_render = now + std::chrono::milliseconds(100);

        int width = 0;
        int height = 0;
        glfwGetFramebufferSize(window, &width, &height);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        const int column_width = std::max(1, width / 3);
        const int last_width = std::max(1, width - 2 * column_width);
        const int row_height = std::max(1, height / 2);
        const int top_height = std::max(1, height - row_height);

        render_view(
            model,
            data,
            option,
            scene,
            context,
            cameras[0],
            {0, row_height, column_width, top_height},
            world_clouds[0],
            transforms[0],
            {1.0F, 0.12F, 0.08F},
            "FRONT CHEST FTX",
            "X RED / Y GREEN / Z BLUE\nYELLOW: ORIGIN + OPTICAL +X");

        render_view(
            model,
            data,
            option,
            scene,
            context,
            cameras[1],
            {column_width, row_height, column_width, top_height},
            world_clouds[1],
            transforms[1],
            {0.1F, 1.0F, 0.25F},
            "REAR CHEST FTX",
            "X RED / Y GREEN / Z BLUE\nYELLOW: ORIGIN + OPTICAL +X");

        render_view(
            model, data, option, scene, context, cameras[2],
            {2 * column_width, row_height, last_width, top_height},
            world_clouds[2], transforms[2], {0.1F, 0.45F, 1.0F},
            "CHASSIS JT16 LOGICAL FRAME",
            "X RED / Y GREEN / Z BLUE (PLATFORM AXES)\n"
            "YELLOW: PACKET AZ=0 CHANNEL 1",
            jt16_reference_ray);

        const std::array<const char*, 3> cloud_titles{
            "FRONT FTX POINT CLOUD", "REAR FTX POINT CLOUD", "JT16 POINT CLOUD"};
        const std::array<std::array<float, 3>, 3> colors{{
            {1.0F, 0.12F, 0.08F},
            {0.1F, 1.0F, 0.25F},
            {0.1F, 0.45F, 1.0F}}};
        for (int index = 0; index < 3; ++index)
        {
            const int view_width = index == 2 ? last_width : column_width;
            render_view(
                model, data, option, scene, context, cameras[3 + index],
                {index * column_width, 0, view_width, row_height},
                world_clouds[index], transforms[index], colors[index],
                cloud_titles[index], "FULL STATIC FRAME + SENSOR ORIGIN",
                index == 2 ? jt16_reference_ray
                           : std::array<double, 3>{1.0, 0.0, 0.0});
        }
        glfwSwapBuffers(window);

        if (now >= next_title)
        {
            const std::string title =
                "Three LiDAR views | red front " + status(clouds[0]) +
                " | green rear " + status(clouds[1]) +
                " | blue JT16 " + status(clouds[2]) +
                (snapshot_built ? " | STATIC SNAPSHOTS" : " | WAITING FOR ALL FRAMES");
            glfwSetWindowTitle(window, title.c_str());
            next_title = now + std::chrono::milliseconds(500);
        }
    }

    running.store(false);
    front.join();
    rear.join();
    chassis.join();
    mjr_freeContext(&context);
    mjv_freeScene(&scene);
    glfwDestroyWindow(window);
    glfwTerminate();
    mj_deleteData(data);
    mj_deleteModel(model);
    return EXIT_SUCCESS;
}
