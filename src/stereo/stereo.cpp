#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include "rclcpp/rclcpp.hpp"
#include "stereo-slam-node.hpp"

#include"System.h"

void write_config_file(sensor_msgs::msg::CameraInfo left_camera_info, sensor_msgs::msg::CameraInfo right_camera_info);

int main(int argc, char **argv)
{
    if(argc < 4)
    {
        std::cerr << "\nUsage: ros2 run orbslam stereo path_to_vocabulary path_to_settings do_rectify" << std::endl;        
        return 1;
    }

    rclcpp::init(argc, argv);

    // malloc error using new.. try shared ptr
    // Create SLAM system. It initializes all system threads and gets ready to process frames
    const bool visualization = true;

    std::string left_info_topic = "/left/camera_info";
    std::string right_info_topic = "/right/camera_info";

    auto right_camera_info = sensor_msgs::msg::CameraInfo();
    auto left_camera_info = sensor_msgs::msg::CameraInfo();
    
    bool left_camera_info_received = false;
    bool right_camera_info_received = false;

    auto info_node = std::make_shared<rclcpp::Node>("camera_info_subscriber");

    left_camera_info_received = rclcpp::wait_for_message<sensor_msgs::msg::CameraInfo>(left_camera_info, info_node,left_info_topic, std::chrono::seconds(5));
    right_camera_info_received = rclcpp::wait_for_message<sensor_msgs::msg::CameraInfo>(right_camera_info, info_node,right_info_topic, std::chrono::seconds(5));

    if(left_camera_info_received && right_camera_info_received){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Camera info received");
    }

    else
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "No camera info provided");

    write_config_file(left_camera_info, right_camera_info);

    const string path_to_vocabulary = "/ws/src/ros2_orbslam3/vocabulary/ORBvoc.txt";
    const string path_to_settings = "/ws/src/ros2_orbslam3/config/stereo/config.yaml";

    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::STEREO, visualization);

    auto node = std::make_shared<StereoSlamNode>(&SLAM, argv[2], argv[3]);

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}

void write_config_file(sensor_msgs::msg::CameraInfo left_camera_info, sensor_msgs::msg::CameraInfo right_camera_info){
    
    ofstream MyFile("/ws/src/ros2_orbslam3/config/stereo/config.yaml");

    MyFile << "%YAML:1.0" << endl;
    MyFile << "File.version: '1.0'" << endl;

    MyFile << "Camera.type: '"<< left_camera_info.distortion_model <<"'"<< endl;

    MyFile << "Camera1.fx: " << left_camera_info.k[0] << endl;
    MyFile << "Camera1.fy: "<< left_camera_info.k[4] << endl;

    MyFile << "Camera1.cx: "<< left_camera_info.k[2] << endl;
    MyFile << "Camera1.cy: "<< left_camera_info.k[5]  << endl;

    /*MyFile << 'Camera1.k1: "'<< left_camera_info.distortion_model <<'"'<< endl;
    MyFile << 'Camera1.k2: "'<< left_camera_info.distortion_model <<'"'<< endl;
    MyFile << 'Camera1.k3: "'<< left_camera_info.distortion_model <<'"'<< endl;

    MyFile << 'Camera1.p1: "'<< right_camera_info.distortion_model <<'"'<< endl;
    MyFile << 'Camera1.p2: "'<< right_camera_info.distortion_model <<'"'<< endl;*/

    MyFile << "Camera2.fx: " << right_camera_info.k[0] << endl;
    MyFile << "Camera2.fy: "<< right_camera_info.k[4] << endl;

    MyFile << "Camera2.cx: "<< right_camera_info.k[2] << endl;
    MyFile << "Camera2.cy: "<< right_camera_info.k[5]  << endl;

    MyFile.close();
}