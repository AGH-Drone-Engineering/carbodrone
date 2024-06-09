#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "Eigen/Geometry"
#include "rclcpp/rclcpp.hpp"
#include "px4_ros_com/frame_transforms.h"
#include "tf2_ros/transform_broadcaster.h"

#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

using std::placeholders::_1;
using namespace px4_msgs::msg;
using namespace nav_msgs::msg;
using namespace geometry_msgs::msg;
using Eigen::Vector3d;
using Eigen::Quaterniond;
using px4_ros_com::frame_transforms::ned_to_enu_local_frame;
using px4_ros_com::frame_transforms::px4_to_ros_orientation;

class OdomPublisher : public rclcpp::Node
{
public:
    OdomPublisher()
        : Node("odom_publisher")
    {
        _odom_pub = this->create_publisher<Odometry>("odometry", rclcpp::SensorDataQoS());
        _odom_sub = this->create_subscription<VehicleOdometry>(
            "/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),
            std::bind(&OdomPublisher::_odom_cb, this, _1));
        _tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

private:
    void _odom_cb(const VehicleOdometry &odom_in)
    {
        rclcpp::Time time = get_clock()->now();

        auto position = ned_to_enu_local_frame(Vector3d(odom_in.position[0], odom_in.position[1], odom_in.position[2]));
        auto orientation = px4_to_ros_orientation(Quaterniond(odom_in.q[0], odom_in.q[1], odom_in.q[2], odom_in.q[3]));

        Odometry odom_out;
        odom_out.header.stamp = time;
        odom_out.header.frame_id = "odom";
        odom_out.child_frame_id = "base_link";

        odom_out.pose.pose.position.x = position.x();
        odom_out.pose.pose.position.y = position.y();
        odom_out.pose.pose.position.z = position.z();

        odom_out.pose.pose.orientation.w = orientation.w();
        odom_out.pose.pose.orientation.x = orientation.x();
        odom_out.pose.pose.orientation.y = orientation.y();
        odom_out.pose.pose.orientation.z = orientation.z();

        _odom_pub->publish(odom_out);

        TransformStamped tf;
        tf.header.stamp = time;
        tf.header.frame_id = "odom";
        tf.child_frame_id = "base_link";

        tf.transform.translation.x = position.x();
        tf.transform.translation.y = position.y();
        tf.transform.translation.z = position.z();

        tf.transform.rotation.w = orientation.w();
        tf.transform.rotation.x = orientation.x();
        tf.transform.rotation.y = orientation.y();
        tf.transform.rotation.z = orientation.z();

        _tf_broadcaster->sendTransform(tf);
    }

    rclcpp::Publisher<Odometry>::SharedPtr _odom_pub;
    rclcpp::Subscription<VehicleOdometry>::SharedPtr _odom_sub;
    std::shared_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomPublisher>());
    rclcpp::shutdown();
    return 0;
}
