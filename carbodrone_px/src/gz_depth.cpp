#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

using std::placeholders::_1;
using namespace sensor_msgs::msg;

class GzDepth : public rclcpp::Node
{
public:
    GzDepth()
        : Node("gz_depth")
    {
        _pub = this->create_publisher<PointCloud2>("pc_out", rclcpp::SensorDataQoS());
        _sub = this->create_subscription<PointCloud2>(
            "pc_in", rclcpp::SensorDataQoS(),
            std::bind(&GzDepth::_cb, this, _1));
    }

private:
    void _cb(const PointCloud2 &pc_in)
    {
        auto time = get_clock()->now();
        auto pc_out = pc_in;
        pc_out.header.stamp = time;
        pc_out.header.frame_id = "OakD-Lite/base_link";
        _pub->publish(pc_out);
    }

    rclcpp::Publisher<PointCloud2>::SharedPtr _pub;
    rclcpp::Subscription<PointCloud2>::SharedPtr _sub;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GzDepth>());
    rclcpp::shutdown();
    return 0;
}
