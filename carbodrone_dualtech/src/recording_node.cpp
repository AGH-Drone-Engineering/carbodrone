#include <chrono>
#include <memory>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"

#include "cv_bridge/cv_bridge.h"
#include "image_geometry/pinhole_camera_model.h"
#include "image_transport/image_transport.hpp"
#include "opencv2/opencv.hpp"

#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include "mavros_msgs/msg/altitude.hpp"

using time_point_t = std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>;

static time_point_t ros2_timestamp_to_chrono(const builtin_interfaces::msg::Time &ros_ts)
{
    long long total_nanoseconds = static_cast<long long>(ros_ts.sec) * 1'000'000'000LL + ros_ts.nanosec;
    std::chrono::nanoseconds duration_ns(total_nanoseconds);
    time_point_t chrono_tp(duration_ns);
    return chrono_tp;
}

static double abs_time_diff(const time_point_t &t1, const time_point_t &t2)
{
    return std::abs(std::chrono::duration<double>(t1 - t2).count());
}

class RecordingNode : public rclcpp::Node
{
public:
    RecordingNode()
        : Node("recording_node")
        , _nh(std::shared_ptr<RecordingNode>(this, [](auto *) {}))
        , _it(_nh)
        , _image_sub(
              _it.subscribe("/camera/image", 1, std::bind(&RecordingNode::image_callback, this, std::placeholders::_1)))
    {
        _mavros_global_position_global_sub = create_subscription<sensor_msgs::msg::NavSatFix>(
            "/mavros/global_position/global", rclcpp::SensorDataQoS(),
            std::bind(&RecordingNode::mavros_global_position_global_callback, this, std::placeholders::_1));

        _mavros_altitude_sub = create_subscription<mavros_msgs::msg::Altitude>(
            "/mavros/altitude", rclcpp::SensorDataQoS(),
            std::bind(&RecordingNode::mavros_altitude_callback, this, std::placeholders::_1));
    }

private:
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &img_in)
    {
        if (_current_image && _current_global_position_global && _current_altitude)
        {
            auto time_img = ros2_timestamp_to_chrono(_current_image->header.stamp);
            auto time_gps = ros2_timestamp_to_chrono(_current_global_position_global->header.stamp);
            auto time_alt = ros2_timestamp_to_chrono(_current_altitude->header.stamp);
            auto time_diff = std::max(abs_time_diff(time_img, time_gps), abs_time_diff(time_img, time_alt));
            if (time_diff > 0.5)
            {
                RCLCPP_WARN(get_logger(), "Image metadata is too old: %f s", time_diff);
                return;
            }
        }
        _current_image = img_in;
    }

    void
    mavros_global_position_global_callback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr &global_position_global_in)
    {
        if (!_current_image || !_current_global_position_global)
        {
            _current_global_position_global = global_position_global_in;
            return;
        }

        auto new_gps_t = ros2_timestamp_to_chrono(global_position_global_in->header.stamp);
        auto current_gps_t = ros2_timestamp_to_chrono(_current_global_position_global->header.stamp);
        auto current_img_t = ros2_timestamp_to_chrono(_current_image->header.stamp);

        if (abs_time_diff(new_gps_t, current_img_t) < abs_time_diff(current_gps_t, current_img_t))
        {
            _current_global_position_global = global_position_global_in;
        }
    }

    void mavros_altitude_callback(const mavros_msgs::msg::Altitude::ConstSharedPtr &altitude_in)
    {
        if (!_current_image || !_current_altitude)
        {
            _current_altitude = altitude_in;
            return;
        }

        auto new_alt_t = ros2_timestamp_to_chrono(altitude_in->header.stamp);
        auto current_alt_t = ros2_timestamp_to_chrono(_current_altitude->header.stamp);
        auto current_img_t = ros2_timestamp_to_chrono(_current_image->header.stamp);

        if (abs_time_diff(new_alt_t, current_img_t) < abs_time_diff(current_alt_t, current_img_t))
        {
            _current_altitude = altitude_in;
        }
    }

    rclcpp::Node::SharedPtr _nh;

    image_transport::ImageTransport _it;
    image_transport::Subscriber _image_sub;

    sensor_msgs::msg::Image::ConstSharedPtr _current_image;
    sensor_msgs::msg::NavSatFix::ConstSharedPtr _current_global_position_global;
    mavros_msgs::msg::Altitude::ConstSharedPtr _current_altitude;

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr _mavros_global_position_global_sub;
    rclcpp::Subscription<mavros_msgs::msg::Altitude>::SharedPtr _mavros_altitude_sub;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RecordingNode>());
    rclcpp::shutdown();
    return 0;
}
