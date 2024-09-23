#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include "rclcpp/rclcpp.hpp"
#include "stereo-inertial-slam-node.hpp"



int main(int argc, char **argv)
{
    

    rclcpp::init(argc, argv);

    // malloc error using new.. try shared ptr
    // Create SLAM system. It initializes all system threads and gets ready to process frames
    

    auto node = std::make_shared<CameraSyncNode>();

    rclcpp::spin(node);


    
    rclcpp::shutdown();

    return 0;
}