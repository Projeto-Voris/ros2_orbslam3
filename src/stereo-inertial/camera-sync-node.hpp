

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




class CameraSyncNode : public rclcpp::Node
{
public:
    CameraSyncNode();

    ~CameraSyncNode();
    

private: 
    using ImageMsg = sensor_msgs::msg::Image;
    using ImuMsg = sensor_msgs::msg::Imu;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> approximate_sync_policy;

    void GrabStereo(const sensor_msgs::msg::Image::SharedPtr msgRGB, const sensor_msgs::msg::Image::SharedPtr msgD);

    bool doRectify;
    cv::Mat M1l,M2l,M1r,M2r;

    cv_bridge::CvImageConstPtr cv_ptrLeft;
    cv_bridge::CvImageConstPtr cv_ptrRight;


    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image> > left_sub;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image> > right_sub;

    std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy> > syncApproximate;

    
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imgpublisher;
};
