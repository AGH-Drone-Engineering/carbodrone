#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "message_filters/time_synchronizer.h"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/latest_time.h"

#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/vehicle_global_position.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

using geometry_msgs::msg::TransformStamped;
using px4_msgs::msg::VehicleGlobalPosition;
using px4_msgs::msg::VehicleStatus;
using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

class GroundPub : public rclcpp::Node
{
public:
    GroundPub()
        : Node("ground_pub")
    {
        rmw_qos_profile_t qos = rmw_qos_profile_sensor_data;

        _status_sub = std::make_shared<message_filters::Subscriber<VehicleStatus>>(
            this,
            "/fmu/out/vehicle_status",
            qos);

        _global_position_sub = std::make_shared<message_filters::Subscriber<VehicleGlobalPosition>>(
            this,
            "/fmu/out/vehicle_global_position",
            qos);

        _time_sync = std::make_shared<message_filters::Synchronizer<sync_policy>>(
            sync_policy(get_clock()),
            *_status_sub,
            *_global_position_sub);

        _time_sync->registerCallback(std::bind(&GroundPub::on_position, this, _1, _2));

        _tf_buf = std::make_shared<tf2_ros::Buffer>(get_clock());
        _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buf);
        _tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

private:
    void on_position(
        const VehicleStatus::ConstSharedPtr &status,
        const VehicleGlobalPosition::ConstSharedPtr &global_position)
    {
        rclcpp::Time time = rclcpp::Time(
            global_position->timestamp / 1000000ULL,
            (global_position->timestamp % 1000000ULL) * 1000ULL,
            get_clock()->get_clock_type());

        if (status->arming_state == VehicleStatus::ARMING_STATE_ARMED && !_home_global_position)
        {
            RCLCPP_INFO(this->get_logger(), "Home position set: %.6f, %.6f, %.2f",
                        global_position->lat, global_position->lon, global_position->alt_ellipsoid);
            _home_global_position = global_position;
        }

        double current_alt = global_position->alt_ellipsoid;
        double home_alt = _home_global_position ? _home_global_position->alt_ellipsoid : current_alt;

        try
        {
            TransformStamped tf;
            tf.header.stamp = time;
            tf.header.frame_id = "odom";
            tf.child_frame_id = "ground";

            double home2drone = current_alt - home_alt;
            double odom2done = _tf_buf->lookupTransform("odom", "base_link", time, 10ms).transform.translation.z;
            double home2odom = home2drone - odom2done;

            tf.transform.translation.z = -home2odom;
            _tf_broadcaster->sendTransform(std::move(tf));
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Transform lookup failed: %s", ex.what());
            return;
        }
    }

    using sync_policy = message_filters::sync_policies::LatestTime<VehicleStatus, VehicleGlobalPosition>;
    std::shared_ptr<message_filters::Synchronizer<sync_policy>> _time_sync;

    std::shared_ptr<message_filters::Subscriber<VehicleStatus>> _status_sub;
    std::shared_ptr<message_filters::Subscriber<VehicleGlobalPosition>> _global_position_sub;

    std::shared_ptr<tf2_ros::Buffer> _tf_buf;
    std::shared_ptr<tf2_ros::TransformListener> _tf_listener;
    std::shared_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;

    VehicleGlobalPosition::ConstSharedPtr _home_global_position;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GroundPub>());
    rclcpp::shutdown();
    return 0;
}
