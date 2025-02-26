#include "rgbd-inertial-node.hpp"
#include <rclcpp/qos.hpp>
#include <opencv2/core/core.hpp>

//SLAM.TrackRGBD(im, depth, timestamp, vImuMeas);

using std::placeholders::_1;

RgbdInertialNode::RgbdInertialNode(ORB_SLAM3::System *SLAM, const string &strDoEqual)
:   Node("ORB_SLAM3_ROS2"),
    SLAM_(SLAM)
{   
    stringstream ss_eq(strDoEqual);
    ss_eq >> boolalpha >> doEqual_;

    auto qos = rclcpp::SensorDataQoS();

    //rgb_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(this, "/camera/camera/color/image_raw");
    //depth_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(this, "/camera/camera/aligned_depth_to_color/image_raw");
    subImu_ = this->create_subscription<ImuMsg>("/camera/camera/imu", qos, std::bind(&RgbdInertialNode::GrabImu, this, _1));
    subImgRgb_ = this->create_subscription<ImageMsg>("/camera/camera/color/image_raw", 100, std::bind(&RgbdInertialNode::GrabImageRgb, this, _1));
    subImgDepth_ = this->create_subscription<ImageMsg>("/camera/camera/aligned_depth_to_color/image_raw", 100, std::bind(&RgbdInertialNode::GrabImageDepth, this, _1));

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/slam/pose", 10);
    syncThread_ = new std::thread(&RgbdInertialNode::SyncWithImu, this);
    
    bClahe_ = doEqual_;
    RCLCPP_INFO(this->get_logger(), "RGBD-inertial Node initialized. CLAHE enabled: %s", bClahe_ ? "true" : "false");
}



RgbdInertialNode::~RgbdInertialNode()
{
    // Delete sync thread
    syncThread_->join();
    delete syncThread_;

    // Stop all threads
    SLAM_->Shutdown();

    // Save camera trajectory
    SLAM_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void RgbdInertialNode::GrabImu(const ImuMsg::SharedPtr msg)
{
    bufMutex_.lock();
    imuBuf_.push(msg);
    bufMutex_.unlock();

    //RCLCPP_INFO(this->get_logger(), "Received IMU data.");
}

void RgbdInertialNode::GrabImageRgb(const ImageMsg::SharedPtr msgRgb)
{
    bufMutexRgb_.lock();

    if (!imgRgbBuf_.empty())
        imgRgbBuf_.pop();
    imgRgbBuf_.push(msgRgb);

    bufMutexRgb_.unlock();
    //RCLCPP_INFO(this->get_logger(), "Received RGB image with timestamp: %ld", msgRgb->header.stamp.sec);
}

void RgbdInertialNode::GrabImageDepth(const ImageMsg::SharedPtr msgDepth)
{
    bufMutexDepth_.lock();

    
    if (!imgDepthBuf_.empty())
        imgDepthBuf_.pop();
    imgDepthBuf_.push(msgDepth);

    bufMutexDepth_.unlock();
    //RCLCPP_INFO(this->get_logger(), "Received Depth image with timestamp: %ld", msgDepth->header.stamp.sec);
}

// im = cv::Mat(cv::Size(width_img, height_img), CV_8UC3, (void*)(color_frame.get_data()), cv::Mat::AUTO_STEP);


cv::Mat RgbdInertialNode::GetRgbImage(const ImageMsg::SharedPtr msg){
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }

    if (cv_ptr->image.type() == CV_8UC3)
    {
        return cv_ptr->image.clone();
    }
    else
    {
        std::cerr << "Error image type RGB" << std::endl;
        return cv_ptr->image.clone();
    }
}

// depth = cv::Mat(cv::Size(width_img, height_img), CV_16U, (void*)(depth_frame.get_data()), cv::Mat::AUTO_STEP);
//https://docs.ros.org/en/jade/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html
cv::Mat RgbdInertialNode::GetDepthImage(const ImageMsg::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }

    if (cv_ptr->image.type() == CV_16UC1)
    {
        return cv_ptr->image.clone();
    }
    else
    {
        std::cerr << "Error image type depth" << std::endl;
        return cv_ptr->image.clone();
    }
}



void RgbdInertialNode::SyncWithImu()
{
    const double maxTimeDiff = 0.01;

    while (1)
    {
        cv::Mat imRgb, imDepth;
        double tImRgb = 0, tImDepth = 0;
        if (!imgRgbBuf_.empty() && !imgDepthBuf_.empty() && !imuBuf_.empty())
        {
            tImRgb = Utility::StampToSec(imgRgbBuf_.front()->header.stamp);
            tImDepth = Utility::StampToSec(imgDepthBuf_.front()->header.stamp);

            bufMutexRgb_.lock();
            while ((tImDepth - tImRgb) > maxTimeDiff && imgRgbBuf_.size() > 1)
            {
                imgRgbBuf_.pop();
                tImRgb = Utility::StampToSec(imgRgbBuf_.front()->header.stamp);
            }
            bufMutexRgb_.unlock();

            if ((tImRgb - tImDepth) > maxTimeDiff || (tImDepth - tImRgb) > maxTimeDiff)
            {
                std::cout << "big time difference" << std::endl;
                continue;
            }
            if (tImRgb > Utility::StampToSec(imuBuf_.back()->header.stamp))
                continue;

            bufMutexRgb_.lock();
            imRgb = GetRgbImage(imgRgbBuf_.front());
            imgRgbBuf_.pop();
            bufMutexRgb_.unlock();

            bufMutexDepth_.lock();
            imDepth = GetDepthImage(imgDepthBuf_.front());
            imgDepthBuf_.pop();
            bufMutexDepth_.unlock();

            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            bufMutex_.lock();
            if (!imuBuf_.empty())
            {
                // Load imu measurements from buffer
                vImuMeas.clear();
                while (!imuBuf_.empty() && Utility::StampToSec(imuBuf_.front()->header.stamp) <= tImRgb)
                {
                    double t = Utility::StampToSec(imuBuf_.front()->header.stamp);
                    cv::Point3f acc(imuBuf_.front()->linear_acceleration.x, imuBuf_.front()->linear_acceleration.y, imuBuf_.front()->linear_acceleration.z);
                    cv::Point3f gyr(imuBuf_.front()->angular_velocity.x, imuBuf_.front()->angular_velocity.y, imuBuf_.front()->angular_velocity.z);
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                    imuBuf_.pop();
                }
            }
            bufMutex_.unlock();

            if (bClahe_)
            {
                clahe_->apply(imRgb, imRgb);
            }


            Sophus::SE3f pose = SLAM_->TrackRGBD(imRgb, imDepth, tImRgb, vImuMeas);

            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header.stamp = rclcpp::Clock().now();
            pose_msg.header.frame_id = "map";
            pose_msg.pose.position.x = pose.translation().x();
            pose_msg.pose.position.y = pose.translation().y();
            pose_msg.pose.position.z = pose.translation().z();

            Eigen::Quaternionf q = pose.unit_quaternion();
            pose_msg.pose.orientation.x = q.x();
            pose_msg.pose.orientation.y = q.y();
            pose_msg.pose.orientation.z = q.z();
            pose_msg.pose.orientation.w = q.w();

            pose_pub_->publish(pose_msg);

            std::chrono::milliseconds tSleep(1);
            std::this_thread::sleep_for(tSleep);
        }
    }
}