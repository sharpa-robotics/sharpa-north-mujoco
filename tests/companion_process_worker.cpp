// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#include <csignal>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>
#include <thread>

namespace fs = std::filesystem;
namespace
{
volatile std::sig_atomic_t Running = 1;

void stop(int)
{
    Running = 0;
}
}  // namespace

int main(int argc, char** argv)
{
    const std::string profile = argc > 1 ? fs::path(argv[argc - 1]).filename().string() : std::string();
    if (profile == "fail") return 7;
    if (profile == "slow")
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        return 9;
    }

    std::signal(SIGTERM, stop);
    std::signal(SIGINT, stop);
    const char* libraries = std::getenv("LD_LIBRARY_PATH");
    const char* configured_root = std::getenv("TEST_COMPANION_ROOT");
    std::cout << fs::current_path().string() << '\n'
              << (libraries ? libraries : "") << '\n'
              << (configured_root ? configured_root : "") << '\n'
              << (argc > 1 ? argv[argc - 1] : "") << '\n'
              << std::flush;
    while (Running) std::this_thread::sleep_for(std::chrono::milliseconds(20));
    return 0;
}
