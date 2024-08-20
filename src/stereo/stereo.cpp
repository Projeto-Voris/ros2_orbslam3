#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include "rclcpp/rclcpp.hpp"
#include "stereo-slam-node.hpp"

#include"System.h"

void write_config_file(sensor_msgs::msg::CameraInfo left_camera_info, sensor_msgs::msg::CameraInfo right_camera_info, const string &orb_extractor_n_features, 
    const string &orb_extractor_scale_factor , const string &orb_extractor_n_levels, const string &orb_extractor_ini_th_fast, const string &orb_extractor_min_th_fast,
    const string &stereo_th_depth);

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
        write_config_file(left_camera_info, right_camera_info, argv[4], argv[5], argv[6], argv[7], argv[8], argv[9]);
    }

    else
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "No camera info provided");


    const string path_to_vocabulary = "/ws/src/ros2_orbslam3/vocabulary/ORBvoc.txt";
    const string path_to_settings = "/ws/src/ros2_orbslam3/config/stereo/config.yaml";

    ORB_SLAM3::System SLAM(argv[1], path_to_settings, ORB_SLAM3::System::STEREO, visualization);

    auto node = std::make_shared<StereoSlamNode>(&SLAM, argv[2], argv[3]);

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}

void write_config_file(sensor_msgs::msg::CameraInfo left_camera_info, sensor_msgs::msg::CameraInfo right_camera_info, const string &orb_extractor_n_features, 
    const string &orb_extractor_scale_factor , const string &orb_extractor_n_levels, const string &orb_extractor_ini_th_fast, const string &orb_extractor_min_th_fast,
    const string &stereo_th_depth){
    
    ofstream MyFile("/ws/src/ros2_orbslam3/config/stereo/config.yaml");

    MyFile << "%YAML:1.0" << endl;
    MyFile << "File.version: \"1.0\"" << endl;

    if(left_camera_info.distortion_model.compare("pinhole") == 0)
        MyFile << "Camera.type: \"PinHole\""<< endl;

    MyFile << "Camera1.fx: " << left_camera_info.k[0] << endl;
    MyFile << "Camera1.fy: "<< left_camera_info.k[4] << endl;

    MyFile << "Camera1.cx: "<< left_camera_info.k[2] << endl;
    MyFile << "Camera1.cy: "<< left_camera_info.k[5]  << endl;

    MyFile << "Camera1.k1: "<< left_camera_info.d[0] << endl;
    MyFile << "Camera1.k2: "<< left_camera_info.d[1]  << endl;
    MyFile << "Camera1.k3: "<< left_camera_info.d[2] << endl;

    MyFile << "Camera1.p1: "<< left_camera_info.d[3] << endl;
    MyFile << "Camera1.p2: "<< left_camera_info.d[4]  << endl;

    MyFile << "Camera2.fx: "<< right_camera_info.k[0] << endl;
    MyFile << "Camera2.fy: "<< right_camera_info.k[4] << endl;

    MyFile << "Camera2.cx: "<< right_camera_info.k[2] << endl;
    MyFile << "Camera2.cy: "<< right_camera_info.k[5]  << endl;

    MyFile << "Camera2.k1: "<< right_camera_info.d[0] << endl;
    MyFile << "Camera2.k2: "<< right_camera_info.d[1]  << endl;
    MyFile << "Camera2.k3: "<< right_camera_info.d[2] << endl;

    MyFile << "Camera2.p1: "<< right_camera_info.d[3] << endl;
    MyFile << "Camera2.p2: "<< right_camera_info.d[4]  << endl;

    MyFile << "Camera.width: "<< right_camera_info.width << endl;
    MyFile << "Camera.height: "<< right_camera_info.height  << endl;

    MyFile << "Camera.fps: 30" << endl;
    MyFile << "Camera.RGB: 1" << endl;

    MyFile << "Stereo.ThDepth:"<< stereo_th_depth.c_str()<< endl;

    float baseline = right_camera_info.p[3];

    MyFile << "Stereo.T_c1_c2: !!opencv-matrix" << endl;
    MyFile << " " << "rows: 4" << endl;
    MyFile << " " << "cols: 4" << endl;
    MyFile << " " << "dt: f" << endl;
    MyFile << " data: [" << right_camera_info.r[0] << "," <<  right_camera_info.r[1] << "," <<  right_camera_info.r[2] << "," <<  baseline << "," << endl;
    MyFile << "     " << right_camera_info.r[3] << "," <<  right_camera_info.r[4] << "," <<  right_camera_info.r[5] << "," <<  right_camera_info.p[7] << "," << endl;
    MyFile << "     " << right_camera_info.r[6] << "," <<  right_camera_info.r[7] << "," <<  right_camera_info.r[8] << "," <<  right_camera_info.p[8] << "," << endl;
    MyFile << "     " << 0 << "," <<  0 << "," <<  0 << "," <<  1 << "]" << endl;

    MyFile << "ORBextractor.nFeatures:" << orb_extractor_n_features.c_str() << endl;
    MyFile << "ORBextractor.scaleFactor:" << orb_extractor_scale_factor.c_str() << endl;
    MyFile << "ORBextractor.nLevels:" << orb_extractor_n_levels.c_str() << endl;
    MyFile << "ORBextractor.iniThFAST:" << orb_extractor_ini_th_fast.c_str() << endl;
    MyFile << "ORBextractor.minThFAST:" << orb_extractor_min_th_fast.c_str() << endl;

    MyFile << "Viewer.KeyFrameSize: 0.05" << endl;
    MyFile << "Viewer.KeyFrameLineWidth: 1.0" << endl;
    MyFile << "Viewer.GraphLineWidth: 0.9" << endl;
    MyFile << "Viewer.PointSize: 2.0" << endl;
    MyFile << "Viewer.CameraSize: 0.08" << endl;
    MyFile << "Viewer.CameraLineWidth: 3.0" << endl;
    MyFile << "Viewer.ViewpointX: 0.0" << endl;
    MyFile << "Viewer.ViewpointY: -0.7" << endl;
    MyFile << "Viewer.ViewpointZ: -1.8" << endl;
    MyFile << "Viewer.ViewpointF: 500.0" << endl;
    MyFile << "Viewer.imageViewScale: 1.0" << endl;

    MyFile.close();
}