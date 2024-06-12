#include "rclcpp/rclcpp.hpp"

#include "Eigen/Geometry"

#include "tf2_ros/transform_broadcaster.h"
#include "px4_ros_com/frame_transforms.h"
#include "px4_msgs/msg/home_position.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

using geometry_msgs::msg::TransformStamped;
using px4_msgs::msg::HomePosition;
using namespace std::chrono_literals;
using px4_ros_com::frame_transforms::ned_to_enu_local_frame;

class HomePub : public rclcpp::Node
{
public:
    HomePub()
        : Node("home_pub")
    {
        _home_sub = create_subscription<HomePosition>(
            "/fmu/out/home_position",
            rclcpp::SensorDataQoS(),
            std::bind(&HomePub::on_home_position, this, std::placeholders::_1));

        _tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        
        _map2odom_tf.header.frame_id = "map";
        _map2odom_tf.child_frame_id = "odom";

        _timer = create_wall_timer(100ms, std::bind(&HomePub::on_timer, this));
    }

private:
    void on_timer()
    {
        _map2odom_tf.header.stamp = get_clock()->now();
        _tf_broadcaster->sendTransform(_map2odom_tf);
    }

    void on_home_position(const HomePosition &home)
    {
        if (!home.valid_lpos)
        {
            RCLCPP_WARN(get_logger(), "Home position not valid");
            return;
        }

        RCLCPP_INFO(get_logger(), "Home position set: %.2f, %.2f, %.2f", home.x, home.y, home.z);

        Eigen::Vector3d ned(home.x, home.y, home.z);
        Eigen::Vector3d enu = ned_to_enu_local_frame(ned);

        _map2odom_tf.transform.translation.x = -enu.x();
        _map2odom_tf.transform.translation.y = -enu.y();
        _map2odom_tf.transform.translation.z = -enu.z();
    }

    rclcpp::Subscription<HomePosition>::SharedPtr _home_sub;
    std::shared_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;
    TransformStamped _map2odom_tf;
    rclcpp::TimerBase::SharedPtr _timer;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HomePub>());
    rclcpp::shutdown();
    return 0;
}
