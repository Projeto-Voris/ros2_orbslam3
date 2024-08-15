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
//  std::string left_topic = this->get_parameter("/stereo/left_cam").as_string();
//  std::string right_topic = this->get_parameter("/stereo/right_cam").as_string();
//  std::string namespace_node = this->get_parameter("/stereo/namespace").as_string();
    std::string left_topic = "/left/image_raw";
    std::string right_topic = "/right/image_raw";

//  left_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(shared_ptr<rclcpp::Node>(this), namespace_node+"/"+left_topic);
    left_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(shared_ptr<rclcpp::Node>(this), left_topic);
    right_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(shared_ptr<rclcpp::Node>(this), right_topic);

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
    /*cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
    cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);*/

    cv::Mat resized_left;
    cv::Mat resized_right;
    cv::resize(cv_ptrLeft->image, resized_left, cv::Size(800,600));
    cv::resize(cv_ptrRight->image, resized_right, cv::Size(800,600));

    Sophus::SE3f SE3 = m_SLAM->TrackStereo(resized_left, resized_right, msgLeft->header.stamp.sec);
    
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
    std::cout << "Service called" << std::endl;

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