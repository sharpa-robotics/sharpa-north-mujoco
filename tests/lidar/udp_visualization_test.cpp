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
#include <string>
#include <thread>
#include <vector>

namespace
{

using sharpa::mujoco::plugins::common::UdpEndpoint;
using sharpa::mujoco::plugins::common::UdpSocket;

constexpr double kPi = 3.14159265358979323846;
constexpr std::size_t kFtxPacketSize = 554;
constexpr std::size_t kFtxChannelOffset = 24;
constexpr std::size_t kFtxTailOffset = 504;
constexpr std::size_t kJtPacketSize = 80;
constexpr std::size_t kMcuEnvelopeSize = 32;

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
    std::size_t frames_in_window = 0;
    double frame_rate_hz = 0.0;
    std::chrono::steady_clock::time_point rate_window_start =
        std::chrono::steady_clock::now();
};

struct TruthGeom
{
    int type = mjGEOM_BOX;
    std::array<float, 2> position{};
    std::array<float, 3> size{};
    float yaw_degrees = 0.0F;
    std::array<float, 3> color{};
};

struct TruthScene
{
    std::vector<TruthGeom> geoms;
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
            received != kFtxPacketSize ||
            buffer[0] != 0xEE || buffer[1] != 0xFF)
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
        for (std::size_t channel = 0; channel < 96; ++channel)
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
            kAzimuthSamples * pattern.jt16_elevations.size(),
            Point{});
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
             channel < pattern.jt16_elevations.size();
             ++channel)
        {
            const std::uint16_t raw =
                sharpa::mujoco::plugins::common::read_le16(
                    packet + 18 + channel * 3);
            if (raw == 0)
            {
                cloud.points[
                    azimuth_index * pattern.jt16_elevations.size() + channel] =
                    Point{};
                continue;
            }
            cloud.points[
                azimuth_index * pattern.jt16_elevations.size() + channel] =
                from_angles(
                    static_cast<double>(raw) * 0.004,
                    azimuth + pattern.jt16_azimuth_offsets[channel],
                    pattern.jt16_elevations[channel] * kPi / 180.0);
        }
    }
}

using Glyph = std::array<std::uint8_t, 7>;

Glyph glyph(char character)
{
    switch (character)
    {
        case 'A': return {14, 17, 17, 31, 17, 17, 17};
        case 'B': return {30, 17, 17, 30, 17, 17, 30};
        case 'C': return {14, 17, 16, 16, 16, 17, 14};
        case 'D': return {30, 17, 17, 17, 17, 17, 30};
        case 'E': return {31, 16, 16, 30, 16, 16, 31};
        case 'F': return {31, 16, 16, 30, 16, 16, 16};
        case 'G': return {14, 17, 16, 23, 17, 17, 15};
        case 'H': return {17, 17, 17, 31, 17, 17, 17};
        case 'I': return {31, 4, 4, 4, 4, 4, 31};
        case 'J': return {7, 2, 2, 2, 18, 18, 12};
        case 'L': return {16, 16, 16, 16, 16, 16, 31};
        case 'N': return {17, 25, 21, 19, 17, 17, 17};
        case 'O': return {14, 17, 17, 17, 17, 17, 14};
        case 'P': return {30, 17, 17, 30, 16, 16, 16};
        case 'R': return {30, 17, 17, 30, 20, 18, 17};
        case 'S': return {15, 16, 16, 14, 1, 1, 30};
        case 'T': return {31, 4, 4, 4, 4, 4, 4};
        case 'U': return {17, 17, 17, 17, 17, 17, 14};
        case 'V': return {17, 17, 17, 17, 17, 10, 4};
        case 'X': return {17, 17, 10, 4, 10, 17, 17};
        case 'Y': return {17, 17, 10, 4, 4, 4, 4};
        case '0': return {14, 17, 19, 21, 25, 17, 14};
        case '1': return {4, 12, 4, 4, 4, 4, 14};
        case '2': return {14, 17, 1, 2, 4, 8, 31};
        case '3': return {30, 1, 1, 14, 1, 1, 30};
        case '4': return {2, 6, 10, 18, 31, 2, 2};
        case '5': return {31, 16, 16, 30, 1, 1, 30};
        case '6': return {6, 8, 16, 30, 17, 17, 14};
        case '7': return {31, 1, 2, 4, 8, 8, 8};
        case '8': return {14, 17, 17, 14, 17, 17, 14};
        case '9': return {14, 17, 17, 15, 1, 2, 12};
        case '.': return {0, 0, 0, 0, 0, 12, 12};
        case '/': return {1, 2, 2, 4, 8, 8, 16};
        case '+': return {0, 4, 4, 31, 4, 4, 0};
        default: return {0, 0, 0, 0, 0, 0, 0};
    }
}

void draw_label(
    int viewport_x,
    int viewport_y,
    int viewport_width,
    int viewport_height,
    const std::string& text)
{
    glViewport(viewport_x, viewport_y, viewport_width, viewport_height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0.0, viewport_width, 0.0, viewport_height, -1.0, 1.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    constexpr float scale = 2.0F;
    constexpr float advance = 12.0F;
    const float label_width =
        18.0F + advance * static_cast<float>(text.size());
    glColor3f(0.025F, 0.03F, 0.04F);
    glBegin(GL_QUADS);
    glVertex2f(5.0F, static_cast<float>(viewport_height) - 25.0F);
    glVertex2f(label_width, static_cast<float>(viewport_height) - 25.0F);
    glVertex2f(label_width, static_cast<float>(viewport_height) - 5.0F);
    glVertex2f(5.0F, static_cast<float>(viewport_height) - 5.0F);
    glEnd();

    glColor3f(0.94F, 0.95F, 0.98F);
    glBegin(GL_QUADS);
    for (std::size_t index = 0; index < text.size(); ++index)
    {
        const Glyph rows = glyph(text[index]);
        for (std::size_t row = 0; row < rows.size(); ++row)
        {
            for (int column = 0; column < 5; ++column)
            {
                if ((rows[row] & (1U << (4 - column))) == 0)
                {
                    continue;
                }
                const float left =
                    10.0F + static_cast<float>(index) * advance +
                    static_cast<float>(column) * scale;
                const float top =
                    static_cast<float>(viewport_height) - 9.0F -
                    static_cast<float>(row) * scale;
                glVertex2f(left, top - scale);
                glVertex2f(left + scale, top - scale);
                glVertex2f(left + scale, top);
                glVertex2f(left, top);
            }
        }
    }
    glEnd();
}

void draw_cloud(
    Cloud& cloud,
    int viewport_x,
    int viewport_y,
    int viewport_width,
    int viewport_height,
    const std::array<float, 3>& color,
    const std::string& label)
{
    glViewport(viewport_x, viewport_y, viewport_width, viewport_height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    const double aspect =
        static_cast<double>(viewport_width) /
        static_cast<double>(viewport_height);
    glOrtho(-8.0 * aspect, 8.0 * aspect, -3.0, 10.0, -50.0, 50.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glRotatef(62.0F, 1.0F, 0.0F, 0.0F);
    glRotatef(-25.0F, 0.0F, 0.0F, 1.0F);

    glColor3f(0.35F, 0.35F, 0.35F);
    glBegin(GL_LINES);
    glVertex3f(-8.0F, 0.0F, 0.0F);
    glVertex3f(8.0F, 0.0F, 0.0F);
    glVertex3f(0.0F, -8.0F, 0.0F);
    glVertex3f(0.0F, 8.0F, 0.0F);
    glEnd();

    glPointSize(2.0F);
    glColor3f(color[0], color[1], color[2]);
    std::size_t valid_points = 0;
    std::size_t total_slots = 0;
    double scan_rate_hz = 0.0;
    glBegin(GL_POINTS);
    {
        std::lock_guard<std::mutex> lock(cloud.mutex);
        total_slots = cloud.points.size();
        scan_rate_hz = cloud.frame_rate_hz;
        for (const Point& point : cloud.points)
        {
            if (point.valid)
            {
                ++valid_points;
                glVertex3f(point.x, point.y, point.z);
            }
        }
    }
    glEnd();

    const int fps_tenths =
        std::max(0, static_cast<int>(std::lround(scan_rate_hz * 10.0)));
    const std::string fps_text =
        std::to_string(fps_tenths / 10) + "." +
        std::to_string(fps_tenths % 10);
    draw_label(
        viewport_x,
        viewport_y,
        viewport_width,
        viewport_height,
        label + " RX FPS " + fps_text + " PTS " +
            std::to_string(valid_points) + "/" +
            std::to_string(total_slots));
}

void draw_box(
    float center_x,
    float center_y,
    float half_x,
    float half_y,
    float angle_degrees,
    const std::array<float, 3>& color)
{
    glPushMatrix();
    glTranslatef(center_x, center_y, 0.0F);
    glRotatef(angle_degrees, 0.0F, 0.0F, 1.0F);
    glColor3f(color[0], color[1], color[2]);
    glBegin(GL_LINE_LOOP);
    glVertex2f(-half_x, -half_y);
    glVertex2f(half_x, -half_y);
    glVertex2f(half_x, half_y);
    glVertex2f(-half_x, half_y);
    glEnd();
    glPopMatrix();
}

void draw_circle(
    float center_x,
    float center_y,
    float radius,
    const std::array<float, 3>& color)
{
    glColor3f(color[0], color[1], color[2]);
    glBegin(GL_LINE_LOOP);
    for (int index = 0; index < 40; ++index)
    {
        const double angle = 2.0 * kPi * static_cast<double>(index) / 40.0;
        glVertex2f(
            center_x + radius * static_cast<float>(std::cos(angle)),
            center_y + radius * static_cast<float>(std::sin(angle)));
    }
    glEnd();
}

bool load_truth_scene(
    const std::string& model_path,
    TruthScene& scene,
    std::string* load_error)
{
    std::array<char, 1024> error{};
    mjModel* model = mj_loadXML(
        model_path.c_str(),
        nullptr,
        error.data(),
        static_cast<int>(error.size()));
    if (!model)
    {
        if (load_error)
        {
            *load_error = error.data();
        }
        return false;
    }
    mjData* data = mj_makeData(model);
    if (!data)
    {
        mj_deleteModel(model);
        if (load_error)
        {
            *load_error = "Could not allocate MuJoCo data for the overview.";
        }
        return false;
    }
    mj_forward(model, data);
    scene.geoms.clear();
    for (int geom = 0; geom < model->ngeom; ++geom)
    {
        const int type = model->geom_type[geom];
        if (model->geom_bodyid[geom] != 0 ||
            (type != mjGEOM_BOX && type != mjGEOM_CYLINDER))
        {
            continue;
        }
        TruthGeom output;
        output.type = type;
        output.position = {
            static_cast<float>(data->geom_xpos[3 * geom]),
            static_cast<float>(data->geom_xpos[3 * geom + 1])};
        output.size = {
            static_cast<float>(model->geom_size[3 * geom]),
            static_cast<float>(model->geom_size[3 * geom + 1]),
            static_cast<float>(model->geom_size[3 * geom + 2])};
        const mjtNum* rotation = data->geom_xmat + 9 * geom;
        output.yaw_degrees = static_cast<float>(
            std::atan2(rotation[3], rotation[0]) * 180.0 / kPi);
        output.color = {
            model->geom_rgba[4 * geom],
            model->geom_rgba[4 * geom + 1],
            model->geom_rgba[4 * geom + 2]};
        scene.geoms.push_back(output);
    }
    mj_deleteData(data);
    mj_deleteModel(model);
    return true;
}

void draw_ground_truth(
    const TruthScene& scene,
    int viewport_x,
    int viewport_y,
    int viewport_width,
    int viewport_height)
{
    glViewport(viewport_x, viewport_y, viewport_width, viewport_height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-4.2, 4.2, -5.6, 5.6, -1.0, 1.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glLineWidth(1.0F);
    glColor3f(0.14F, 0.16F, 0.20F);
    glBegin(GL_LINES);
    for (int coordinate = -5; coordinate <= 5; ++coordinate)
    {
        glVertex2f(-4.2F, static_cast<float>(coordinate));
        glVertex2f(4.2F, static_cast<float>(coordinate));
        if (coordinate >= -4 && coordinate <= 4)
        {
            glVertex2f(static_cast<float>(coordinate), -5.6F);
            glVertex2f(static_cast<float>(coordinate), 5.6F);
        }
    }
    glEnd();

    for (const TruthGeom& geom : scene.geoms)
    {
        if (geom.type == mjGEOM_BOX)
        {
            draw_box(
                geom.position[0],
                geom.position[1],
                geom.size[0],
                geom.size[1],
                geom.yaw_degrees,
                geom.color);
        }
        else
        {
            draw_circle(
                geom.position[0],
                geom.position[1],
                geom.size[0],
                geom.color);
        }
    }

    draw_label(
        viewport_x,
        viewport_y,
        viewport_width,
        viewport_height,
        "STATIC SCENE OVERVIEW");
}

std::string cloud_status(Cloud& cloud)
{
    std::lock_guard<std::mutex> lock(cloud.mutex);
    const std::size_t valid_points = static_cast<std::size_t>(std::count_if(
        cloud.points.begin(),
        cloud.points.end(),
        [](const Point& point) { return point.valid; }));
    const int fps_tenths =
        std::max(0, static_cast<int>(std::lround(cloud.frame_rate_hz * 10.0)));
    return std::to_string(valid_points) + " PTS " +
           std::to_string(fps_tenths / 10) + "." +
           std::to_string(fps_tenths % 10) + " HZ";
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
        std::cerr << "Usage: " << argv[0]
                  << " --config <path> [--model <path>]\n";
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
    std::vector<sharpa::mujoco::simulator::LidarMount> mounts;
    if (!sharpa::mujoco::simulator::parse_lidar_mounts(
            config.devices, mounts, &error))
    {
        std::cerr << error << '\n';
        return EXIT_FAILURE;
    }
    const auto find_mount = [&](const std::string& name) {
        return std::find_if(
            mounts.begin(),
            mounts.end(),
            [&](const auto& mount) { return mount.instance.name == name; });
    };
    const auto front_mount = find_mount("chest_lidar_front");
    const auto rear_mount = find_mount("chest_lidar_rear");
    const auto chassis_mount = find_mount("chassis_lidar_front");
    if (front_mount == mounts.end() || rear_mount == mounts.end() ||
        chassis_mount == mounts.end())
    {
        std::cerr << "Configuration must enable chest_lidar_front, "
                     "chest_lidar_rear, and chassis_lidar_front.\n";
        return EXIT_FAILURE;
    }

    if (!glfwInit())
    {
        return 1;
    }
    GLFWwindow* window = glfwCreateWindow(
        1280,
        900,
        "LiDAR UDP comparison with ground truth",
        nullptr,
        nullptr);
    if (!window)
    {
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    std::atomic<bool> running{true};
    sharpa::mujoco::plugins::lidar::ScanPattern ftx_pattern;
    sharpa::mujoco::plugins::lidar::ScanPattern jt16_pattern;
    std::string calibration_error;
    if (!sharpa::mujoco::plugins::lidar::load_ftx140_scan_pattern(
            front_mount->instance.calibration_file,
            ftx_pattern,
            &calibration_error) ||
        !sharpa::mujoco::plugins::lidar::load_jt16_scan_pattern(
            chassis_mount->instance.calibration_file,
            jt16_pattern,
            &calibration_error))
    {
        std::cerr << calibration_error << '\n';
        glfwDestroyWindow(window);
        glfwTerminate();
        return EXIT_FAILURE;
    }

    TruthScene truth_scene;
    if (!load_truth_scene(config.model_path, truth_scene, &error))
    {
        std::cerr << error << '\n';
        glfwDestroyWindow(window);
        glfwTerminate();
        return EXIT_FAILURE;
    }
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
    auto next_title_update = std::chrono::steady_clock::now();

    while (!glfwWindowShouldClose(window))
    {
        int width = 0;
        int height = 0;
        glfwGetFramebufferSize(window, &width, &height);
        glClearColor(0.03F, 0.035F, 0.045F, 1.0F);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        const int view_width = std::max(1, width / 2);
        const int view_height = std::max(1, height / 2);
        draw_cloud(
            clouds[0],
            0,
            view_height,
            view_width,
            height - view_height,
            {1.0F, 0.25F, 0.2F},
            "FRONT FTX140");
        draw_cloud(
            clouds[1],
            view_width,
            view_height,
            width - view_width,
            height - view_height,
            {0.2F, 1.0F, 0.35F},
            "REAR FTX140");
        draw_cloud(
            clouds[2],
            0,
            0,
            view_width,
            view_height,
            {0.25F, 0.55F, 1.0F},
            "CHASSIS JT16");
        draw_ground_truth(
            truth_scene,
            view_width,
            0,
            width - view_width,
            view_height);
        glfwSwapBuffers(window);
        glfwPollEvents();
        const auto now = std::chrono::steady_clock::now();
        if (now >= next_title_update)
        {
            const std::string title =
                "LiDAR UDP receiver | front FTX " +
                cloud_status(clouds[0]) + " | rear FTX " +
                cloud_status(clouds[1]) + " | chassis JT16 " +
                cloud_status(clouds[2]);
            glfwSetWindowTitle(window, title.c_str());
            std::cout << title << std::endl;
            next_title_update = now + std::chrono::milliseconds(500);
        }
        if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        {
            glfwSetWindowShouldClose(window, GLFW_TRUE);
        }
    }

    running.store(false);
    front.join();
    rear.join();
    chassis.join();
    glfwDestroyWindow(window);
    glfwTerminate();
    return EXIT_SUCCESS;
}
