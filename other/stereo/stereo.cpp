#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "stereo-slam-node.hpp"
#include "System.h"

int main(int argc, char **argv)
{
    if (argc < 4)
    {
        std::cerr << "\nUsage: ros2 run orbslam stereo path_to_vocabulary path_to_settings do_rectify" << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);

    bool visualization = true;
    ORB_SLAM3::System pSLAM(argv[1], argv[2], ORB_SLAM3::System::STEREO, visualization);

    auto node = std::make_shared<StereoSlamNode>(&pSLAM, argv[2], argv[3]);
    RCLCPP_INFO(node->get_logger(), "ORB-SLAM3 Node Started");

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}

