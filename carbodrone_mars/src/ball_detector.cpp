#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "image_geometry/pinhole_camera_model.h"
#include "image_transport/image_transport.hpp"

#include "px4_ros_com/frame_transforms.h"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"

#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "px4_msgs/msg/landing_target_pose.hpp"

using namespace std::chrono_literals;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using px4_ros_com::frame_transforms::enu_to_ned_local_frame;

class BallDetector : public rclcpp::Node
{
public:
    BallDetector()
        : Node("ball_detector")
        , _nh(std::shared_ptr<BallDetector>(this, [](auto*){}))
        , _it(_nh)
        , _camera_sub(_it.subscribeCamera(
            "/camera/image",
            1,
            std::bind(
                &BallDetector::image_callback,
                this,
                std::placeholders::_1,
                std::placeholders::_2)))
    {
        _tf_buf = std::make_shared<tf2_ros::Buffer>(get_clock());
        _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buf);
        _tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        _target_pub = create_publisher<px4_msgs::msg::LandingTargetPose>(
            "/fmu/in/landing_target_pose", rclcpp::SensorDataQoS());
    }

private:
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &img_in, const sensor_msgs::msg::CameraInfo::ConstSharedPtr &cam_info)
    {
        rclcpp::Time time = img_in->header.stamp;

        double dist_bottom;
        try
        {
            auto t = _tf_buf->lookupTransform(
                "ground", img_in->header.frame_id, time, 10ms);
            dist_bottom = t.transform.translation.z;
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not get height above ground");
            return;
        }

        auto img_ptr = cv_bridge::toCvShare(img_in, "bgr8");
        auto img = img_ptr->image;

        cv::Mat img_hsv;
        cv::cvtColor(img, img_hsv, cv::COLOR_BGR2HSV);

        cv::Mat threshold;
        cv::inRange(img_hsv, cv::Scalar(27, 100, 79), cv::Scalar(34, 255, 255), threshold);

        cv::Mat labels, stats, centroids;
        int numComponents = cv::connectedComponentsWithStats(threshold, labels, stats, centroids);

        double img_cx = img.cols / 2;
        double img_cy = img.rows / 2;
        double best_x;
        double best_y;
        double best_area = -1;
        double best_distance = 1e9;
        for (int i = 1; i < numComponents; ++i)
        {
            double area = stats.at<int>(i, cv::CC_STAT_AREA);
            double x = centroids.at<double>(i, 0);
            double y = centroids.at<double>(i, 1);
            double distance = std::sqrt((x - img_cx) * (x - img_cx) + (y - img_cy) * (y - img_cy));
            if (distance < best_distance)
            {
                best_x = x;
                best_y = y;
                best_area = area;
                best_distance = distance;
            }
        }

        if (best_area < 0)
            return;

        _cam_model.fromCameraInfo(cam_info);

        cv::Point2d uv_raw(best_x, best_y);
        auto uv = _cam_model.rectifyPoint(uv_raw);
        auto ray = _cam_model.projectPixelTo3dRay(uv);

        geometry_msgs::msg::TransformStamped cam2target;
        cam2target.header.stamp = time;
        cam2target.header.frame_id = img_in->header.frame_id;
        cam2target.child_frame_id = "landing_target";

        cam2target.transform.translation.x = ray.x * dist_bottom;
        cam2target.transform.translation.y = ray.y * dist_bottom;
        cam2target.transform.translation.z = ray.z * dist_bottom;

        _tf_broadcaster->sendTransform(std::move(cam2target));

        try
        {
            auto local2target = _tf_buf->lookupTransform(
                "odom", "landing_target", cam2target.header.stamp, 30ms);

            auto target_pos = enu_to_ned_local_frame(Vector3d(
                local2target.transform.translation.x,
                local2target.transform.translation.y,
                local2target.transform.translation.z));

            px4_msgs::msg::LandingTargetPose target;
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
    }

    rclcpp::Node::SharedPtr _nh;

    rclcpp::Publisher<px4_msgs::msg::LandingTargetPose>::SharedPtr _target_pub;

    image_transport::ImageTransport _it;
    image_transport::CameraSubscriber _camera_sub;

    std::shared_ptr<tf2_ros::Buffer> _tf_buf;
    std::shared_ptr<tf2_ros::TransformListener> _tf_listener;
    std::shared_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;

    image_geometry::PinholeCameraModel _cam_model;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BallDetector>());
    rclcpp::shutdown();
    return 0;
}
