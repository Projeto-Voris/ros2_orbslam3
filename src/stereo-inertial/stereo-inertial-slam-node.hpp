#ifndef __STEREO_SLAM_NODE_HPP__
#define __STEREO_SLAM_NODE_HPP__

#include <iostream>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"

#include "std_srvs/srv/trigger.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "sensor_msgs/msg/camera_info.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/transform.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include <cv_bridge/cv_bridge.h>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"
#include "MapPoint.h"



class StereoSlamNode : public rclcpp::Node
{
public:
    StereoSlamNode(ORB_SLAM3::System* pSLAM, const string &strSettingsFile, const string &strDoRectify);

    ~StereoSlamNode();

    void SavePointCloudSRV(std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res);
    void PublishPointCloud();
    

private: 
    using ImageMsg = sensor_msgs::msg::Image;
    using ImuMsg = sensor_msgs::msg::Imu;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> approximate_sync_policy;

    void GrabStereo(const sensor_msgs::msg::Image::SharedPtr msgImg);
    void GrabIMU(const ImuMsg::SharedPtr msgImu);
    void TimerCallback();
    
    ORB_SLAM3::System* m_SLAM;

    bool doRectify;
    cv::Mat M1l,M2l,M1r,M2r;

    cv_bridge::CvImageConstPtr cv_ptrImage;

    sensor_msgs::msg::Imu::SharedPtr imu_message;
    
    std::vector<ORB_SLAM3::IMU::Point> vImu;

    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Image> > image_sub;
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Imu> > imu_sub;

    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy> > syncApproximate;

    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr publisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pclpublisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imgpublisher;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_pcl_srv;
    rclcpp::CallbackGroup::SharedPtr sub_cb_group_;
    rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
};

#endif
