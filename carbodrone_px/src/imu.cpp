#include "Eigen/Geometry"
#include "rclcpp/rclcpp.hpp"
#include "px4_ros_com/frame_transforms.h"

#include "sensor_msgs/msg/imu.hpp"
#include "px4_msgs/msg/sensor_combined.hpp"

using std::placeholders::_1;
using namespace px4_msgs::msg;
using namespace sensor_msgs::msg;
using Eigen::Vector3d;
using Eigen::Quaterniond;
using px4_ros_com::frame_transforms::ned_to_enu_local_frame;
using px4_ros_com::frame_transforms::px4_to_ros_orientation;
using px4_ros_com::frame_transforms::aircraft_to_baselink_body_frame;

class ImuNode : public rclcpp::Node
{
public:
    ImuNode()
        : Node("imu")
    {
        _imu_pub = this->create_publisher<Imu>("imu", rclcpp::SensorDataQoS());
        _sensor_sub = this->create_subscription<SensorCombined>(
            "/fmu/out/sensor_combined", rclcpp::SensorDataQoS(),
            std::bind(&ImuNode::_sensor_cb, this, _1));
    }

private:
    void _sensor_cb(const SensorCombined &sensor)
    {
        rclcpp::Time time = get_clock()->now();

        auto ang_vel = aircraft_to_baselink_body_frame(Vector3d(
            sensor.gyro_rad[0], sensor.gyro_rad[1], sensor.gyro_rad[2]));
        auto lin_acc = aircraft_to_baselink_body_frame(Vector3d(
            sensor.accelerometer_m_s2[0], sensor.accelerometer_m_s2[1], sensor.accelerometer_m_s2[2]));
        // auto orientation = px4_to_ros_orientation(Quaterniond(
        //     odom_in.q[0], odom_in.q[1], odom_in.q[2], odom_in.q[3]));

        Imu imu;
        imu.header.stamp = time;
        imu.header.frame_id = "base_link";
        
        imu.angular_velocity.x = ang_vel.x();
        imu.angular_velocity.y = ang_vel.y();
        imu.angular_velocity.z = ang_vel.z();

        imu.linear_acceleration.x = lin_acc.x();
        imu.linear_acceleration.y = lin_acc.y();
        imu.linear_acceleration.z = lin_acc.z();

        imu.orientation_covariance[0] = -1;

        _imu_pub->publish(std::move(imu));
    }

    rclcpp::Publisher<Imu>::SharedPtr _imu_pub;
    rclcpp::Subscription<SensorCombined>::SharedPtr _sensor_sub;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuNode>());
    rclcpp::shutdown();
    return 0;
}
