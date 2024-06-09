#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "px4_ros_com/frame_transforms.h"
#include "tf2_ros/transform_broadcaster.h"

#include "sensor_msgs/msg/image.hpp"
#include "px4_msgs/msg/landing_target_pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

using std::placeholders::_1;
using namespace px4_msgs::msg;
using namespace sensor_msgs::msg;
using namespace geometry_msgs::msg;
using Eigen::Vector3d;
using Eigen::Quaterniond;
using px4_ros_com::frame_transforms::enu_to_ned_local_frame;

class BallDetector : public rclcpp::Node
{
public:
    BallDetector()
        : Node("ball_detector")
    {
        _tf_buf = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        _tf_listener = std::make_unique<tf2_ros::TransformListener>(*_tf_buf);
        _img_sub = this->create_subscription<Image>(
            "/bottom_camera", rclcpp::SensorDataQoS(),
            std::bind(&BallDetector::_img_cb, this, _1));
        _target_pub = this->create_publisher<LandingTargetPose>(
            "/fmu/in/landing_target_pose", rclcpp::SensorDataQoS());
        _tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    }

private:
    void _img_cb(const Image::ConstSharedPtr &img_in)
    {
        auto img_ptr = cv_bridge::toCvCopy(img_in, "bgr8");
        auto img = img_ptr->image(cv::Rect(
            (img_ptr->image.cols - img_ptr->image.rows) / 2, 0,
            img_ptr->image.rows, img_ptr->image.rows));

        cv::Mat img_hsv;
        cv::cvtColor(img, img_hsv, cv::COLOR_BGR2HSV);

        cv::Mat threshold;
        cv::inRange(img_hsv, cv::Scalar(27, 100, 79), cv::Scalar(34, 255, 255), threshold);

        auto moments = cv::moments(threshold, true);
        auto cx = moments.m10 / moments.m00;
        auto cy = moments.m01 / moments.m00;
        auto radius = std::sqrt(moments.m00 / 3.141);

        try
        {
            auto ground2cam = _tf_buf->lookupTransform(
                "odom", img_in->header.frame_id, tf2::TimePointZero);

            TransformStamped cam2target;
            cam2target.header = img_in->header;
            cam2target.child_frame_id = "landing_target";

            cam2target.transform.translation.x = ground2cam.transform.translation.z;
            cam2target.transform.translation.y = -((cx / img.cols) - 0.5);
            cam2target.transform.translation.z = -((cy / img.rows) - 0.5);

            _tf_broadcaster->sendTransform(std::move(cam2target));
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not get ground2cam transform");
        }

        try
        {
            auto local2target = _tf_buf->lookupTransform(
                "odom", "landing_target", tf2::TimePointZero);

            auto target_pos = enu_to_ned_local_frame(Vector3d(
                local2target.transform.translation.x,
                local2target.transform.translation.y,
                local2target.transform.translation.z));

            LandingTargetPose target;
            target.timestamp = static_cast<uint64_t>(local2target.header.stamp.sec) * 1000000ULL + static_cast<uint64_t>(local2target.header.stamp.nanosec) / 1000ULL;
            target.abs_pos_valid = true;
            target.rel_pos_valid = false;
            target.rel_vel_valid = false;
            target.is_static = true;
            target.x_abs = target_pos.x();
            target.y_abs = target_pos.y();
            target.z_abs = target_pos.z();
            _target_pub->publish(std::move(target));
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not get local2target transform");
        }

        cv::circle(img, cv::Point(cx, cy), radius, cv::Scalar(0, 0, 255), 4);

        cv::imshow("detector", img);
        cv::waitKey(1);
    }

    rclcpp::Subscription<Image>::SharedPtr _img_sub;
    rclcpp::Publisher<LandingTargetPose>::SharedPtr _target_pub;
    std::unique_ptr<tf2_ros::Buffer> _tf_buf;
    std::unique_ptr<tf2_ros::TransformListener> _tf_listener;
    std::unique_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BallDetector>());
    rclcpp::shutdown();
    return 0;
}
