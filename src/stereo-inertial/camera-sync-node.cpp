#include "stereo-inertial-slam-node.hpp"

#include<opencv2/core/core.hpp>
#include<string>
#include "System.h"

using std::placeholders::_1;
using std::placeholders::_2;

CameraSyncNode::CameraSyncNode(ORB_SLAM3::System* pSLAM, const string &strSettingsFile, const string &strDoRectify)
:   Node("orbslam"),
    m_SLAM(pSLAM)
{

    std::string left_image_topic = "/left/image_raw";
    std::string right_image_topic = "/right/image_raw";

    std::string left_info_topic = "/left/image_raw";
    std::string right_info_topic = "/right/image_raw";

    auto image_options = rclcpp::SubscriptionOptions();
    image_options.callback_group = sub_cb_group_;

    left_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(shared_ptr<rclcpp::Node>(this), left_image_topic, image_options);
    right_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(shared_ptr<rclcpp::Node>(this), right_image_topic, image_options);
    
    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(10), *left_sub, *right_sub);
    syncApproximate->registerCallback(&CameraSyncNode::GrabStereo, this);

}

CameraSyncNode::~CameraSyncNode()
{
}

void CameraSyncNode::GrabStereo(const ImageMsg::SharedPtr msgLeft, const ImageMsg::SharedPtr msgRight)
{

    RCLCPP_INFO(this->get_logger(), "Image received"); 
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

    cv:Mat matArray[] = {cv_ptrLeft->image,cv_ptrRight->image};
    cv::hconcat(matArray, 2, out);
    
    sensor_msgs::msg::Image imgmsg;
    publisher.publish()
   
}