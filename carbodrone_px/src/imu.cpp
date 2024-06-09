#include "Eigen/Geometry"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "px4_ros_com/frame_transforms.h"

#include "message_filters/time_synchronizer.h"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/latest_time.h"

#include "sensor_msgs/msg/imu.hpp"
#include "px4_msgs/msg/sensor_combined.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
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
        rmw_qos_profile_t qos = rmw_qos_profile_sensor_data;
        _attitude_sub = std::make_shared<message_filters::Subscriber<VehicleAttitude>>(
                          this,
                          "/fmu/out/vehicle_attitude",
                          qos);
        _sensor_sub = std::make_shared<message_filters::Subscriber<SensorCombined>>(
                          this,
                          "/fmu/out/sensor_combined",
                          qos);
        _time_sync = std::make_shared<message_filters::Synchronizer<latest_policy>>(
            latest_policy(this->get_clock()),
            *_attitude_sub,
            *_sensor_sub
        );
        _time_sync->registerCallback(std::bind(&ImuNode::_cb, this, _1, _2));
        _imu_pub = this->create_publisher<Imu>("imu", rclcpp::SensorDataQoS());
    }

private:
    void _cb(const VehicleAttitude::ConstSharedPtr &attitude, const SensorCombined::ConstSharedPtr &sensor)
    {
        rclcpp::Time time_att(attitude->timestamp / 1000000ULL, (attitude->timestamp % 1000000ULL) * 1000ULL);
        rclcpp::Time time_sen(sensor->timestamp / 1000000ULL, (sensor->timestamp % 1000000ULL) * 1000ULL);

        auto time = (time_att > time_sen) ? time_att : time_sen;

        auto ang_vel = aircraft_to_baselink_body_frame(Vector3d(
            sensor->gyro_rad[0], sensor->gyro_rad[1], sensor->gyro_rad[2]));
        auto lin_acc = aircraft_to_baselink_body_frame(Vector3d(
            sensor->accelerometer_m_s2[0], sensor->accelerometer_m_s2[1], sensor->accelerometer_m_s2[2]));
        auto orientation = px4_to_ros_orientation(Quaterniond(
            attitude->q[0], attitude->q[1], attitude->q[2], attitude->q[3]));

        Imu imu;
        imu.header.stamp = time;
        imu.header.frame_id = "base_link";
        
        imu.angular_velocity.x = ang_vel.x();
        imu.angular_velocity.y = ang_vel.y();
        imu.angular_velocity.z = ang_vel.z();

        imu.linear_acceleration.x = lin_acc.x();
        imu.linear_acceleration.y = lin_acc.y();
        imu.linear_acceleration.z = lin_acc.z();

        imu.orientation.w = orientation.w();
        imu.orientation.x = orientation.x();
        imu.orientation.y = orientation.y();
        imu.orientation.z = orientation.z();

        _imu_pub->publish(std::move(imu));
    }

    using latest_policy = message_filters::sync_policies::LatestTime<VehicleAttitude, SensorCombined>;
    std::shared_ptr<message_filters::Synchronizer<latest_policy>> _time_sync;

    std::shared_ptr<message_filters::Subscriber<VehicleAttitude>> _attitude_sub;
    std::shared_ptr<message_filters::Subscriber<SensorCombined>> _sensor_sub;

    rclcpp::Publisher<Imu>::SharedPtr _imu_pub;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuNode>());
    rclcpp::shutdown();
    return 0;
}
