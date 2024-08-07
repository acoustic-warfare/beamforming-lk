/**
 * @file main.cpp
 * @author Tuva, Irreq
 * @date 2024-08-06
 */
#include <ctime>
#include <iomanip>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <sstream>

#include "argparse/argparse.hpp"
#include "aw_control_unit/aw_control_unit.h"

#define UDP_ADDRESS "10.0.0.1"

void setupArgumentParser(argparse::ArgumentParser& program) {
    program.add_argument("--camera")
            .default_value(std::string("false"))
            .help("Camera option, default is false");

    program.add_argument("--ip-address")
            .default_value(std::string(UDP_ADDRESS))
            .help("IP address");

    program.add_argument("--audio")
            .default_value(0)
            .scan<'i', int>()
            .help("Audio option, port nr");

    program.add_argument("--mimo")
            .default_value(false)
            .implicit_value(true)
            .help("MIMO option");

    program.add_argument("--record")
            .default_value(false)
            .implicit_value(true)
            .help("Record option");

    program.add_argument("--mimo-res")
            .default_value(100)
            .scan<'i', int>()
            .help("MIMO Resolution");

    program.add_argument("--tracking")
            .default_value(false)
            .implicit_value(true)
            .help("Tracking option");

    program.add_argument("--verbose")
            .default_value(false)
            .implicit_value(true)
            .help("Program output");

    program.add_argument("--fov")
            .default_value(static_cast<float>(180.0f))
            .scan<'g', float>()
            .help("Field of view");

    program.add_argument("--fps")
            .default_value(false)
            .implicit_value(true)
            .help("Program FPS");

    program.add_argument("--logo")
            .default_value(false)
            .implicit_value(true)
            .help("Use logo");

    program.add_argument("--debug")
            .default_value(false)
            .implicit_value(true)
            .help("Debug mode");

    program.add_argument("--port")
            .append()
            .scan<'i', int>()
            .help("PORT numbers");

    program.add_argument("--wara-ps")
            .default_value(false)
            .implicit_value(true)
            .help("Use WARA PS");

    program.add_argument("--miso")
            .default_value(false)
            .implicit_value(true)
            .help("MISO Mode, needs to be combined with --audio 'portnr'");
}

int main(int argc, char* argv[]) {
    argparse::ArgumentParser program(argv[0]);
    setupArgumentParser(program);
    try {
        program.parse_args(argc, argv);
    } catch (const std::exception& err) {
        std::cerr << err.what() << std::endl;
        std::cerr << program;
        std::exit(1);
    }

    // Retrieve the parsed arguments
    std::string camera = program.get<std::string>("--camera");
    std::string ip_address = program.get<std::string>("--ip-address");
    int audio = program.get<int>("--audio");
    bool mimo = program.get<bool>("--mimo");
    bool tracking = program.get<bool>("--tracking");
    float fov = program.get<float>("--fov");
    int mimo_res = program.get<int>("--mimo-res");
    bool record = program.get<bool>("--record");
    std::vector<int> ports = program.get<std::vector<int>>("--port");
    bool verbose = program.get<bool>("--verbose");
    bool use_camera = (camera.compare("false") != 0);
    bool use_fps = program.get<bool>("--fps");
    bool use_logo = program.get<bool>("--logo");
    bool debug = program.get<bool>("--debug");
    bool use_waraps = program.get<bool>("--wara-ps");
    bool miso = program.get<bool>("--miso");

    if (verbose) {
        std::cout << "Camera: " << camera << std::endl;
        std::cout << "IP Address: " << ip_address << std::endl;
        std::cout << "Audio: " << audio << std::endl;
        std::cout << "MIMO: " << mimo << std::endl;
        std::cout << "Tracking: " << tracking << std::endl;
        std::cout << "FOV: " << fov << std::endl;
        std::cout << "Ports: ";
        for (const auto& port: ports) std::cout << port << " ";
        std::cout << std::endl;
        std::cout << "Use camera: " << use_camera << std::endl;
        std::cout << "Use logo: " << use_logo << std::endl;
        std::cout << "Use fps: " << use_fps << std::endl;
        std::cout << "Debug: " << debug << std::endl;
        std::cout << "WARA PS: " << use_waraps << std::endl;
        std::cout << "Miso: " << miso << std::endl;
    }

    AWControlUnit awControlUnit(use_waraps);
    awControlUnit.Start(ports, ip_address, use_camera, camera, audio,
                        mimo, tracking, mimo_res, verbose, record, fov,
                        use_fps, use_logo, debug, miso);

    return 0;
}
