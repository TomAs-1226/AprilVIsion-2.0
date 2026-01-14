/**
 * @file sim_main.cpp
 * @brief Mac Simulator main entry point
 *
 * FRC Vision Mac Simulator v2.0
 * 2024 CRESCENDO Field Layout
 *
 * This is the desktop simulation/test harness for the FRC vision system.
 * It allows testing pose correction, tracking, and auto-align logic
 * on a MacBook without requiring actual robot hardware.
 */

#include "simulator.hpp"
#include <iostream>
#include <string>
#include <filesystem>
#include <getopt.h>
#include <signal.h>

#ifdef __APPLE__
#include <mach-o/dyld.h>
#endif

namespace fs = std::filesystem;

static frc_vision::sim::Simulator* g_simulator = nullptr;

void signal_handler(int /*sig*/) {
    std::cout << "\n[Sim] Shutdown requested..." << std::endl;
    if (g_simulator) {
        g_simulator->shutdown();
    }
}

void print_usage(const char* prog) {
    std::cout << "FRC Vision Mac Simulator v2.0\n"
              << "2024 CRESCENDO Field Layout\n\n"
              << "Usage: " << prog << " [OPTIONS] [config_path]\n\n"
              << "Options:\n"
              << "  --config, -c PATH    Path to sim_config.yml (default: config/sim_config.yml)\n"
              << "  --no-webcam          Disable webcam (synthetic only)\n"
              << "  --help, -h           Show this help\n\n"
              << "Examples:\n"
              << "  " << prog << "\n"
              << "  " << prog << " --config config/sim_config.yml\n"
              << "  " << prog << " --no-webcam\n";
}

int main(int argc, char* argv[]) {
    // Parse command line arguments
    std::string config_path;
    bool use_webcam = true;

    static struct option long_options[] = {
        {"config", required_argument, nullptr, 'c'},
        {"no-webcam", no_argument, nullptr, 'n'},
        {"help", no_argument, nullptr, 'h'},
        {nullptr, 0, nullptr, 0}
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "c:nh", long_options, nullptr)) != -1) {
        switch (opt) {
            case 'c':
                config_path = optarg;
                break;
            case 'n':
                use_webcam = false;
                break;
            case 'h':
                print_usage(argv[0]);
                return 0;
            default:
                print_usage(argv[0]);
                return 1;
        }
    }

    // Remaining argument is config path
    if (optind < argc && config_path.empty()) {
        config_path = argv[optind];
    }

    // Find config file
    if (config_path.empty()) {
        // Try to find relative to executable
        fs::path exe_path;
        try {
            #ifdef __APPLE__
            // macOS: use _NSGetExecutablePath or realpath
            char path[1024];
            uint32_t size = sizeof(path);
            if (_NSGetExecutablePath(path, &size) == 0) {
                exe_path = fs::canonical(path).parent_path();
            }
            #else
            exe_path = fs::canonical("/proc/self/exe").parent_path();
            #endif
        } catch (...) {
            exe_path = ".";
        }

        // Try several paths
        std::vector<fs::path> search_paths = {
            exe_path / "config" / "sim_config.yml",
            exe_path / ".." / "config" / "sim_config.yml",
            "config/sim_config.yml",
            "../config/sim_config.yml"
        };

        for (const auto& p : search_paths) {
            if (fs::exists(p)) {
                config_path = p.string();
                break;
            }
        }

        // If still not found, use a default path (will create default config)
        if (config_path.empty()) {
            config_path = "config/sim_config.yml";
        }
    }

    // Setup signal handlers
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // Create and run simulator
    frc_vision::sim::Simulator simulator;
    g_simulator = &simulator;

    // Override webcam setting if --no-webcam was specified
    // (config will be modified after load)

    if (!simulator.initialize(config_path)) {
        std::cerr << "[Sim] Failed to initialize simulator" << std::endl;
        return 1;
    }

    int result = simulator.run();

    g_simulator = nullptr;
    return result;
}
