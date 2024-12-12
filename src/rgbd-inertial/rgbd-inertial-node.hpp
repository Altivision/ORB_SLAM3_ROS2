#ifndef __RGBD_INERTIAL_NODE_HPP__
#define __RGBD_INERTIAL_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <cv_bridge/cv_bridge.h>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"

using ImuMsg = sensor_msgs::msg::Imu;
using ImageMsg = sensor_msgs::msg::Image;

class RgbdInertialNode : public rclcpp::Node
{
public:
    RgbdInertialNode(ORB_SLAM3::System* pSLAM, const string &strSettingsFile, const string &strDoRectify, const string &strDoEqual);
    ~RgbdInertialNode();

private:
    void GrabImu(const ImuMsg::SharedPtr msg);Rgbd
    void GrabImageRgb(const ImageMsg::SharedPtr msgLeft);
    void GrabImageDepth(const ImageMsg::SharedPtr msgRight);
    cv::Mat GetImage(const ImageMsg::SharedPtr msg);
    void SyncWithImu();

    rclcpp::Subscription<ImuMsg>::SharedPtr   subImu_;
    rclcpp::Subscription<ImageMsg>::SharedPtr subImgRgb_;
    rclcpp::Subscription<ImageMsg>::SharedPtr subImgDepth_;

    ORB_SLAM3::System *SLAM_;
    std::thread *syncThread_;

    // IMU
    queue<ImuMsg::SharedPtr> imuBuf_;
    std::mutex bufMutex_;

    // Image
    queue<ImageMsg::SharedPtr> imgRgbBuf_, imgDepthBuf_;
    std::mutex bufMutexRgb_, bufMutexDepth_;
};

#endif
