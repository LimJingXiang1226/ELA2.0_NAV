#include "stereo-slam-node.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

using std::placeholders::_1;
using std::placeholders::_2;

StereoSlamNode::StereoSlamNode(ORB_SLAM3::System* pSLAM, const std::string &strSettingsFile, const std::string &strDoRectify)
: Node("ORB_SLAM3_ROS2"), m_SLAM(pSLAM)
{
    std::stringstream ss(strDoRectify);
    ss >> std::boolalpha >> doRectify;

    if (doRectify) {
        cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
        if (!fsSettings.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "ERROR: Wrong path to settings");
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

        if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
            rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0) {
            RCLCPP_ERROR(this->get_logger(), "ERROR: Calibration parameters to rectify stereo are missing!");
            assert(0);
        }

        cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0, 3).colRange(0, 3), cv::Size(cols_l, rows_l), CV_32F, M1l, M2l);
        cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0, 3).colRange(0, 3), cv::Size(cols_r, rows_r), CV_32F, M1r, M2r);
    }

    left_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(this, "camera/camera/infra1/image_rect_raw");
    right_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(this, "camera/camera/infra2/image_rect_raw");

    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy>>(approximate_sync_policy(10), *left_sub, *right_sub);
    syncApproximate->registerCallback(&StereoSlamNode::GrabStereo, this);

    pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("orbslam_pose", 10);
}

StereoSlamNode::~StereoSlamNode()
{
    m_SLAM->Shutdown();
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void StereoSlamNode::GrabStereo(const ImageMsg::SharedPtr msgLeft, const ImageMsg::SharedPtr msgRight)
{
    try {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    Sophus::SE3f cam_pose;
    if (doRectify) {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image, imLeft, M1l, M2l, cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image, imRight, M1r, M2r, cv::INTER_LINEAR);
        cam_pose = m_SLAM->TrackStereo(imLeft, imRight, Utility::StampToSec(msgLeft->header.stamp));
    }
    else {
        cam_pose = m_SLAM->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image, Utility::StampToSec(msgLeft->header.stamp));
    }

    // Convert the pose to ROS message and publish it
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = msgLeft->header.stamp;
    pose_msg.header.frame_id = "map";

    Eigen::Quaternionf q(cam_pose.unit_quaternion());
    pose_msg.pose.orientation.w = q.w();
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();

    Eigen::Vector3f t = cam_pose.translation();
    pose_msg.pose.position.x = t.x();
    pose_msg.pose.position.y = t.y();
    pose_msg.pose.position.z = t.z();

    pose_pub->publish(pose_msg);
}

