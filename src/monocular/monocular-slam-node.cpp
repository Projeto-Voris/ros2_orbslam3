#include "monocular-slam-node.hpp"

#include<opencv2/core/core.hpp>

using std::placeholders::_1;

MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System* pSLAM)
:   Node("orbslam")
{
    m_SLAM = pSLAM;
    // std::cout << "slam changed" << std::endl;
    m_image_subscriber = this->create_subscription<ImageMsg>(
        "sync_camera/image",
        10,
        std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1));
    publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("orbslam/pose", 10);
    std::cout << "slam changed" << std::endl;
}

MonocularSlamNode::~MonocularSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();
}

void MonocularSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    try
    {
        m_cvImPtr = cv_bridge::toCvCopy(msg);
        
        cv::resize(m_cvImPtr->image, resize, cv::Size(), 0.5, 0.5);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    auto sendmsg = geometry_msgs::msg::PoseStamped();
    Sophus::SE3f SE3 = m_SLAM->TrackMonocular(resize, static_cast<double>(msg->header.stamp.sec) + static_cast<double>(msg->header.stamp.nanosec) * 1e-9);
    //std::cout<<std::setprecision (15)<<static_cast<double>(msg->header.stamp.sec) + static_cast<double>(msg->header.stamp.nanosec) * 1e-9<<std::endl;
    sendmsg.header.stamp = this->get_clock()->now();
    sendmsg.header.frame_id = "map";

    sendmsg.pose.position.x = SE3.params()(4);
    sendmsg.pose.position.y = SE3.params()(5);
    sendmsg.pose.position.z = SE3.params()(6);

    sendmsg.pose.orientation.x = SE3.params()(0);
    sendmsg.pose.orientation.y = SE3.params()(1);
    sendmsg.pose.orientation.z = SE3.params()(2);
    sendmsg.pose.orientation.w = SE3.params()(3);

    publisher->publish(sendmsg);
    std::cout<<"Parameter:"<< SE3.angleX()<< std::endl;


}
