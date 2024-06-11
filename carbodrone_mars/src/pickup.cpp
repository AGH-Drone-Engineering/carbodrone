#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
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
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;
using Eigen::Vector3d;
using Eigen::Quaterniond;
using px4_ros_com::frame_transforms::enu_to_ned_local_frame;

enum class State
{
    WARMUP = 0,
    LANDING,
    DONE,
};

class PickupNode : public rclcpp::Node
{
public:
    PickupNode()
        : Node("pickup_node")
    {
        _tf_buf = std::make_unique<tf2_ros::Buffer>(get_clock());
        _tf_listener = std::make_unique<tf2_ros::TransformListener>(*_tf_buf);
        _tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        rmw_qos_profile_t qos = rmw_qos_profile_sensor_data;

        _img_sub = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::Image>>(
            this,
            "/camera/image",
            qos);

        _position_sub = std::make_unique<message_filters::Subscriber<px4_msgs::msg::VehicleLocalPosition>>(
            this,
            "/fmu/out/vehicle_local_position",
            qos);

        _time_sync = std::make_unique<message_filters::Synchronizer<latest_policy>>(
            latest_policy(get_clock()),
            *_img_sub,
            *_position_sub);

        _time_sync->registerCallback(std::bind(&PickupNode::image_callback, this, _1, _2));

        _status_sub = create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status",
            rclcpp::SensorDataQoS(),
            std::bind(&PickupNode::on_vehicle_status, this, _1));

        _target_pub = create_publisher<px4_msgs::msg::LandingTargetPose>(
            "/fmu/in/landing_target_pose", rclcpp::SensorDataQoS());

        _command_pub = create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", rclcpp::ServicesQoS());

        _loop_timer = create_wall_timer(100ms, std::bind(&PickupNode::loop, this));
    }

private:
    void loop()
    {
        switch (_state)
        {
        case State::WARMUP:
            if (!_target_pose_ok) break;
            trigger_precision_landing();
            _state = State::LANDING;
            break;

        case State::LANDING:
            if (_is_landed)
            {
                RCLCPP_INFO(get_logger(), "Landed");
                _state = State::DONE;
            }
            break;

        case State::DONE:
            rclcpp::shutdown();
            break;
        }
    }

    void trigger_precision_landing()
    {
        RCLCPP_INFO(get_logger(), "Triggering precision landing");
        px4_msgs::msg::VehicleCommand msg;
        msg.timestamp = get_clock()->now().nanoseconds() / 1000;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.command = msg.VEHICLE_CMD_NAV_PRECLAND;
        _command_pub->publish(std::move(msg));
    }

    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &img_in, const px4_msgs::msg::VehicleLocalPosition::ConstSharedPtr &vehicle_position)
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

        geometry_msgs::msg::TransformStamped cam2target;
        cam2target.header = img_in->header;
        cam2target.child_frame_id = "landing_target";

        cam2target.transform.translation.x = xm;
        cam2target.transform.translation.y = ym;
        cam2target.transform.translation.z = vehicle_position->dist_bottom;

        _tf_broadcaster->sendTransform(std::move(cam2target));

        try
        {
            auto local2target = _tf_buf->lookupTransform(
                "odom", "landing_target", cam2target.header.stamp, 10ms);

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

            _target_pose_ok = true;
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not get local2target transform");
        }
    }

    void on_vehicle_status(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
    {
        if (msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED)
        {
            _is_landed = true;
        }
    }

    using latest_policy = message_filters::sync_policies::LatestTime<sensor_msgs::msg::Image, px4_msgs::msg::VehicleLocalPosition>;
    std::unique_ptr<message_filters::Synchronizer<latest_policy>> _time_sync;

    std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> _img_sub;
    std::unique_ptr<message_filters::Subscriber<px4_msgs::msg::VehicleLocalPosition>> _position_sub;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr _status_sub;

    rclcpp::Publisher<px4_msgs::msg::LandingTargetPose>::SharedPtr _target_pub;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr _command_pub;

    std::unique_ptr<tf2_ros::Buffer> _tf_buf;
    std::unique_ptr<tf2_ros::TransformListener> _tf_listener;
    std::unique_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;

    rclcpp::TimerBase::SharedPtr _loop_timer;
    State _state = State::WARMUP;
    bool _is_landed = false;
    bool _target_pose_ok = false;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PickupNode>());
    rclcpp::shutdown();
    return 0;
}
