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

    std::string left_image_topic = "/left/image_raw";
    std::string right_image_topic = "/right/image_raw";

    std::string left_info_topic = "/left/image_raw";
    std::string right_info_topic = "/right/image_raw";

    auto right_camera_info = sensor_msgs::msg::CameraInfo();
    auto left_camera_info = sensor_msgs::msg::CameraInfo();
    
    bool left_camera_info_received = false;
    bool right_camera_info_received = false;

    auto info_node = std::make_shared<rclcpp::Node>("camera_info_subscriber");

    left_camera_info_received = rclcpp::wait_for_message<sensor_msgs::msg::CameraInfo>(left_camera_info, info_node,left_info_topic, std::chrono::seconds(1));
    right_camera_info_received = rclcpp::wait_for_message<sensor_msgs::msg::CameraInfo>(right_camera_info, info_node,right_info_topic, std::chrono::seconds(1));

    if(left_camera_info_received && right_camera_info_received)
        write_config_file(left_camera_info, right_camera_info);
    else
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "No camera info provided");

    const string path_to_vocabulary = "/ws/src/ros2_orbslam3/vocabulary/ORBvoc.txt";
    const string path_to_settings = "/ws/src/ros2_orbslam3/config/stereo/config.yaml";

    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::STEREO, visualization);

    auto node = std::make_shared<StereoSlamNode>(&SLAM, argv[2], argv[3]);

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}

void write_config_file(sensor_msgs::msg::CameraInfo left_camera_info, sensor_msgs::msg::CameraInfo right_camera_info){
    //TODO
    /*ofstream MyFile("/ws/src/ros2_orbslam3/config/stereo/config.yaml");

    MyFile << '%YAML:1.0' << endl;
    MyFile << 'File.version: "1.0"' << endl;

    MyFile << 'Camera.type: "'<< cam_info.distortion_model <<'"'<< endl;

    MyFile << 'Camera1.fx: "'<< cam_info.distortion_model <<'"'<< endl;
    MyFile << 'Camera1.fy: "'<< cam_info.distortion_model <<'"'<< endl;

    MyFile << 'Camera1.cx: "'<< cam_info.distortion_model <<'"'<< endl;
    MyFile << 'Camera1.cy:"'<< cam_info.distortion_model <<'"'<< endl;

    MyFile << 'Camera1.k1: "'<< cam_info.distortion_model <<'"'<< endl;
    MyFile << 'Camera1.k2: "'<< cam_info.distortion_model <<'"'<< endl;
    MyFile << 'Camera1.k3: "'<< cam_info.distortion_model <<'"'<< endl;

    MyFile << 'Camera2.p1: "'<< cam_info.distortion_model <<'"'<< endl;
    MyFile << 'Camera2.p2: "'<< cam_info.distortion_model <<'"'<< endl;

    MyFile << 'Camera2.p1: "'<< cam_info.distortion_model <<'"'<< endl;
    MyFile << 'Camera2.p2: "'<< cam_info.distortion_model <<'"'<< endl;

    MyFile.close();*/
}