#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "px4_ros_com/frame_transforms.h"
#include "tf2_ros/transform_broadcaster.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/latest_time.h"

#include "sensor_msgs/msg/image.hpp"
#include "px4_msgs/msg/landing_target_pose.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
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
        _tf_buf = std::make_unique<tf2_ros::Buffer>(get_clock());
        _tf_listener = std::make_unique<tf2_ros::TransformListener>(*_tf_buf);
        _tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        rmw_qos_profile_t qos = rmw_qos_profile_sensor_data;

        _img_sub = std::make_unique<message_filters::Subscriber<Image>>(
            this,
            "/bottom_camera",
            qos);

        _position_sub = std::make_unique<message_filters::Subscriber<VehicleLocalPosition>>(
            this,
            "/fmu/out/vehicle_local_position",
            qos);

        _time_sync = std::make_unique<message_filters::Synchronizer<latest_policy>>(
            latest_policy(get_clock()),
            *_img_sub,
            *_position_sub);

        _time_sync->registerCallback(std::bind(&BallDetector::_cb, this, _1, _2));

        _target_pub = this->create_publisher<LandingTargetPose>(
            "/fmu/in/landing_target_pose", rclcpp::SensorDataQoS());
    }

private:
    void _cb(const Image::ConstSharedPtr &img_in, const VehicleLocalPosition::ConstSharedPtr &vehicle_position)
    {
        auto img_ptr = cv_bridge::toCvCopy(img_in, "bgr8");
        auto img = img_ptr->image(cv::Rect(
            (img_ptr->image.cols - img_ptr->image.rows) / 2, 0,
            img_ptr->image.rows, img_ptr->image.rows));

        cv::Mat img_hsv;
        cv::cvtColor(img, img_hsv, cv::COLOR_BGR2HSV);

        cv::Mat threshold;
        cv::inRange(img_hsv, cv::Scalar(27, 100, 79), cv::Scalar(34, 255, 255), threshold);

        cv::Mat labels, stats, centroids;
        int numComponents = cv::connectedComponentsWithStats(threshold, labels, stats, centroids);

        double img_cx = img.cols * 0.5;
        double img_cy = img.rows * 0.5;
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

        if (best_area < 0) return;

        double xp = best_x - img.cols * 0.5;
        double yp = best_y - img.rows * 0.5;

        double bm = vehicle_position->dist_bottom * (4.5 / 8.0);
        double xm = xp / (img.cols * 0.5) * bm;
        double ym = yp / (img.rows * 0.5) * bm;

        auto radius = std::sqrt(best_area / 3.141);

        TransformStamped cam2target;
        cam2target.header = img_in->header;
        cam2target.child_frame_id = "landing_target";

        cam2target.transform.translation.x = vehicle_position->dist_bottom;
        cam2target.transform.translation.y = -xm;
        cam2target.transform.translation.z = -ym;

        _tf_broadcaster->sendTransform(std::move(cam2target));

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

        cv::circle(img, cv::Point(best_x, best_y), radius, cv::Scalar(0, 0, 255), 4);

        cv::imshow("detector", img);
        cv::waitKey(1);
    }

    using latest_policy = message_filters::sync_policies::LatestTime<Image, VehicleLocalPosition>;
    std::unique_ptr<message_filters::Synchronizer<latest_policy>> _time_sync;

    std::unique_ptr<message_filters::Subscriber<Image>> _img_sub;
    std::unique_ptr<message_filters::Subscriber<VehicleLocalPosition>> _position_sub;

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
