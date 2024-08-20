#include "stereo-slam-node.hpp"

#include<opencv2/core/core.hpp>
#include<string>
#include "System.h"

using std::placeholders::_1;
using std::placeholders::_2;

StereoSlamNode::StereoSlamNode(ORB_SLAM3::System* pSLAM, const string &strSettingsFile, const string &strDoRectify)
:   Node("orbslam"),
    m_SLAM(pSLAM)
{
    std::string left_image_topic = "/left/image_raw";
    std::string right_image_topic = "/right/image_raw";

    std::string left_info_topic = "/left/image_raw";
    std::string right_info_topic = "/right/image_raw";

    left_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(shared_ptr<rclcpp::Node>(this), left_image_topic);
    right_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(shared_ptr<rclcpp::Node>(this), right_image_topic);
    
    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(10), *left_sub, *right_sub);
    syncApproximate->registerCallback(&StereoSlamNode::GrabStereo, this);



    publisher = this->create_publisher<geometry_msgs::msg::TransformStamped>("transform", 10);
    pclpublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 10);
    imgpublisher = this->create_publisher<sensor_msgs::msg::Image>("img_keypoints", 10);

    save_pcl_srv = this->create_service<std_srvs::srv::Trigger>("save_pcl",std::bind(&StereoSlamNode::SavePointCloudSRV, this, std::placeholders::_1, std::placeholders::_2));

}

StereoSlamNode::~StereoSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();
}

void StereoSlamNode::GrabStereo(const ImageMsg::SharedPtr msgLeft, const ImageMsg::SharedPtr msgRight)
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

    cv::Mat Tcw;
    cv::Mat imKey;
    bool debug = false;

    auto sendmsg = geometry_msgs::msg::TransformStamped();
    
    sensor_msgs::msg::Image imgmsg;
    

    Sophus::SE3f SE3 = m_SLAM->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image, msgLeft->header.stamp.sec);
    
    std::vector<cv::KeyPoint> keypoints = m_SLAM->GetTrackedKeyPointsUn();

    
    cv_bridge::CvImage img_bridge;
    sensor_msgs::msg::Image img_msg;

    sendmsg.header.stamp = msgLeft->header.stamp;
    sendmsg.header.frame_id = "base_link";
    sendmsg.child_frame_id = "sm2_left_cam_link";

    sendmsg.transform.translation.x = SE3.params()(4);
    sendmsg.transform.translation.y = SE3.params()(5);
    sendmsg.transform.translation.z = SE3.params()(6);

    sendmsg.transform.rotation.x = SE3.params()(0);
    sendmsg.transform.rotation.y = SE3.params()(1);
    sendmsg.transform.rotation.z = SE3.params()(2);
    sendmsg.transform.rotation.w = SE3.params()(3);

    
    if(debug){

        PublishPointCloud();
        
        cv::drawKeypoints(cv_ptrLeft->image, keypoints, imKey,cv::Scalar(0,255,0), cv::DrawMatchesFlags::DEFAULT);
        cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", imKey).toImageMsg(imgmsg);

        imgpublisher->publish(imgmsg);
    }
    publisher->publish(sendmsg);
   
}

void StereoSlamNode::PublishPointCloud(){
    std::vector<int> indexes;
    std::vector<ORB_SLAM3::MapPoint*> points = m_SLAM->GetTrackedMapPoints();
    auto pointcloudmsg = sensor_msgs::msg::PointCloud2();

    int count = 0;
    
    for (size_t i = 0; i < points.size(); i++)
    {
        if(points[i] != 0){
            count++;
            indexes.push_back(i);

        }
    }
    
    pointcloudmsg.header.stamp = this->get_clock()->now();
    pointcloudmsg.header.frame_id = "map";
    pointcloudmsg.height = 1;
    pointcloudmsg.width = count;
    pointcloudmsg.is_dense = true;
    pointcloudmsg.fields.resize(3);

    // Populate the fields
    pointcloudmsg.fields[0].name = "x";
    pointcloudmsg.fields[0].offset = 0;
    pointcloudmsg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pointcloudmsg.fields[0].count = 1;

    pointcloudmsg.fields[1].name = "y";
    pointcloudmsg.fields[1].offset = 4;
    pointcloudmsg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pointcloudmsg.fields[1].count = 1;

    pointcloudmsg.fields[2].name = "z";
    pointcloudmsg.fields[2].offset = 8;
    pointcloudmsg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pointcloudmsg.fields[2].count = 1;

    pointcloudmsg.point_step = 12; // Size of a single point in bytes (3 floats * 4 bytes/float)
    pointcloudmsg.row_step = pointcloudmsg.point_step * pointcloudmsg.width;
    pointcloudmsg.is_bigendian = true;
    pointcloudmsg.data.resize(pointcloudmsg.point_step*count);

    for (size_t i = 0; i < count; i++)
    {
        float x = points[indexes[i]]->GetWorldPos()(0);
        float y = points[indexes[i]]->GetWorldPos()(1);
        float z = -points[indexes[i]]->GetWorldPos()(2);

        memcpy(&pointcloudmsg.data[i*12], &x, 4);
        memcpy(&pointcloudmsg.data[i*12 + 4], &y, 4);
        memcpy(&pointcloudmsg.data[i*12 + 8], &z, 4);
    }
    pclpublisher->publish(pointcloudmsg);
}

void StereoSlamNode::SavePointCloudSRV(std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res){
    std::vector<ORB_SLAM3::MapPoint*> points = m_SLAM->GetTrackedMapPoints();
    ofstream MyFile("/ws/pcl_file.csv");

    MyFile << 'x,y,z' << endl;

    for (size_t i = 0; i < points.size(); i++)
    {
        if(points[i] != 0){
            float x = points[i]->GetWorldPos()(0);
            float y = points[i]->GetWorldPos()(2);
            float z = -points[i]->GetWorldPos()(1);

            // Write to the file
            MyFile << x << ',' << y << ',' << z << endl;

        }
    }
    // Close the file
    MyFile.close();
}