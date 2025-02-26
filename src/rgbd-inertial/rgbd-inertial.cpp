#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rgbd-inertial-node.hpp"

#include "System.h"

int main(int argc, char **argv)
{
    if(argc < 3)
    {
        std::cerr << "\nUsage: ros2 run orbslam stereo path_to_vocabulary path_to_settings [do_equalize]" << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    if(argc == 3)
    {
        argv[3] = "false"; // no need to do equal
    }

    rclcpp::init(argc, argv);

    // malloc error using new.. try shared ptr
    // Create SLAM system. It initializes all system threads and gets ready to process frames.

    bool visualization = true;
    ORB_SLAM3::System pSLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_RGBD, visualization);

    auto node = std::make_shared<RgbdInertialNode>(&pSLAM, argv[3]);
    std::cout << "============================" << std::endl;

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}