#include "camera-sync-node.hpp"

#include<opencv2/core/core.hpp>
#include<string>

using std::placeholders::_1;
using std::placeholders::_2;

CameraSyncNode::CameraSyncNode()
:   Node("camera_sync")
{

    std::string left_image_topic = "/sm2_left/image_raw";
    std::string right_image_topic = "/sm2_right/image_raw";

    std::string left_info_topic = "/left/image_raw";
    std::string right_info_topic = "/right/image_raw";

    

    left_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(std::shared_ptr<rclcpp::Node>(this), left_image_topic);
    right_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(std::shared_ptr<rclcpp::Node>(this), right_image_topic);
    
    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(10), *left_sub, *right_sub);
    syncApproximate->registerCallback(&CameraSyncNode::GrabStereo, this);

    imgpublisher = this->create_publisher<sensor_msgs::msg::Image>("img", 10);

}

CameraSyncNode::~CameraSyncNode()
{
}

void CameraSyncNode::GrabStereo(const ImageMsg::SharedPtr msgLeft, const ImageMsg::SharedPtr msgRight)
{ 
    // Copy the ros rgb image message to cv::Mat.
    try
    {
         cv_ptrLeft = cv_bridge::toCvShare(msgLeft); 
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Copy the ros depth image message to cv::Mat.
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat out;

    cv::Mat matArray[] = {cv_ptrLeft->image,cv_ptrRight->image};
    cv::hconcat(matArray, 2, out);
    
    sensor_msgs::msg::Image msg;
    cv_bridge::CvImage(std_msgs::msg::Header(), msgLeft->encoding, out).toImageMsg(msg);
    msg.header.stamp.sec = msgLeft->header.stamp.sec;
    msg.header.stamp.nanosec = msgLeft->header.stamp.nanosec;


    imgpublisher->publish(msg);
   
}