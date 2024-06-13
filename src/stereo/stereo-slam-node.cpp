#include "stereo-slam-node.hpp"

#include<opencv2/core/core.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

StereoSlamNode::StereoSlamNode(ORB_SLAM3::System* pSLAM, const string &strSettingsFile, const string &strDoRectify)
:   Node("orbslam"),
    m_SLAM(pSLAM)
{
    /*stringstream ss(strDoRectify);
    ss >> boolalpha >> doRectify;

    if (true){

        cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
        if(!fsSettings.isOpened()){
            cerr << "ERROR: Wrong path to settings" << endl;
            assert(0);
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];
        cout << "Trueeeeeeeee" << endl;
        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0){
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            assert(0);
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);
    }*/

    left_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(shared_ptr<rclcpp::Node>(this), "left_camera/image_raw");
    right_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(shared_ptr<rclcpp::Node>(this), "right_camera/image_raw");

    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(10), *left_sub, *right_sub);
    syncApproximate->registerCallback(&StereoSlamNode::GrabStereo, this);
    publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("orbslam/pose", 10);
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
    
  

    cv::Mat imLeft, imRight;
    auto sendmsg = geometry_msgs::msg::PoseStamped();
    /*cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
    cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);*/

    Sophus::SE3f SE3 = m_SLAM->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image, msgLeft->header.stamp.sec);

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
   
}
