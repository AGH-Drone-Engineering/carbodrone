#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "Eigen/Geometry"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "image_geometry/pinhole_camera_model.h"
#include "image_transport/image_transport.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "mission_params.hpp"

using namespace std::chrono_literals;
using Eigen::Vector3d;

#define MISSION_STATE_LIST \
    X(INIT)                \
    \
    X(DO_TAKEOFF)          \
    \
    X(DO_RTL_REPOSITION)       \
    X(DO_RTL_REPOSITION_DELAY) \
    X(DO_RTL_LAND)             \
    \
    X(DONE) \
    X(NOOP)

enum class MissionState
{
#define X(name) name,
    MISSION_STATE_LIST
#undef X
};

static const char *MissionStateName[] = {
#define X(name) #name,
    MISSION_STATE_LIST
#undef X
};

#include "state_machine_node.hpp"

class MissionNode : public StateMachineNode
{
public:
    MissionNode()
        : StateMachineNode("mission_node")
        , _nh(std::shared_ptr<MissionNode>(this, [](auto*){}))
        , _it(_nh)
        , _image_sub(_it.subscribe("/camera/image", 1, std::bind(&MissionNode::image_callback, this, std::placeholders::_1)))
    {
        _tf_buf = std::make_unique<tf2_ros::Buffer>(get_clock());
        _tf_listener = std::make_unique<tf2_ros::TransformListener>(*_tf_buf);
        _tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    }

private:
    void process_state(MissionState state) override
    {
        switch (state)
        {
        case MissionState::INIT:
            change_state_after_condition(MissionState::DO_TAKEOFF, [this](){
                return _is_armed;
            });
            break;

        case MissionState::DO_TAKEOFF:
            do_takeoff(LANDING_PAD_HOVER_ALTITUDE);

            _takeoff_completed = false;
            change_state_after_condition(MissionState::DONE, [this](){
                return _takeoff_completed;
            });
            break;

        case MissionState::DONE:
            RCLCPP_INFO(get_logger(), "Mission completed");
            rclcpp::shutdown();
            break;

        case MissionState::NOOP:
            break;

        default:
            RCLCPP_ERROR(get_logger(), "Invalid state");
            rclcpp::shutdown();
            break;
        }
    }

    void do_arm()
    {
        RCLCPP_INFO(get_logger(), "[CMD] Arming vehicle");
        // TODO: Implement
    }

    void do_takeoff(float altitude)
    {
        RCLCPP_INFO(get_logger(), "[CMD] Taking off to %.2f meters", altitude);
        // TODO: Implement
    }

    bool landing_completed()
    {
        return !_is_armed;
    }

    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &img_in)
    {
        _current_image = img_in;
    }

    std::unique_ptr<image_geometry::PinholeCameraModel> build_camera_model(const sensor_msgs::msg::Image::ConstSharedPtr &img_in)
    {
        auto img_ptr = cv_bridge::toCvShare(img_in, "bgr8");
        const auto &img = img_ptr->image;

        double img_cx = img.cols * 0.5;
        double img_cy = img.rows * 0.5;

        sensor_msgs::msg::CameraInfo cam_info;
        cam_info.header = img_in->header;
        cam_info.height = img.rows;
        cam_info.width = img.cols;
        cam_info.distortion_model = "plumb_bob";

        cam_info.d.resize(5);
        cam_info.d[0] = 0.0;
        cam_info.d[1] = 0.0;
        cam_info.d[2] = 0.0;
        cam_info.d[3] = 0.0;
        cam_info.d[4] = 0.0;

        cam_info.k[0] = CAMERA_FOCAL_LENGTH_PX;
        cam_info.k[1] = 0.0;
        cam_info.k[2] = img_cx;
        cam_info.k[3] = 0.0;
        cam_info.k[4] = CAMERA_FOCAL_LENGTH_PX;
        cam_info.k[5] = img_cy;
        cam_info.k[6] = 0.0;
        cam_info.k[7] = 0.0;
        cam_info.k[8] = 1.0;

        cam_info.r[0] = 1.0;
        cam_info.r[1] = 0.0;
        cam_info.r[2] = 0.0;
        cam_info.r[3] = 0.0;
        cam_info.r[4] = 1.0;
        cam_info.r[5] = 0.0;
        cam_info.r[6] = 0.0;
        cam_info.r[7] = 0.0;
        cam_info.r[8] = 1.0;

        cam_info.p[0] = CAMERA_FOCAL_LENGTH_PX;
        cam_info.p[1] = 0.0;
        cam_info.p[2] = img_cx;
        cam_info.p[3] = 0.0;
        cam_info.p[4] = 0.0;
        cam_info.p[5] = CAMERA_FOCAL_LENGTH_PX;
        cam_info.p[6] = img_cy;
        cam_info.p[7] = 0.0;
        cam_info.p[8] = 0.0;
        cam_info.p[9] = 0.0;
        cam_info.p[10] = 1.0;
        cam_info.p[11] = 0.0;

        cam_info.binning_x = 0;
        cam_info.binning_y = 0;
        cam_info.roi.x_offset = 0;
        cam_info.roi.y_offset = 0;
        cam_info.roi.height = 0;
        cam_info.roi.width = 0;
        cam_info.roi.do_rectify = false;

        auto cam_model = std::make_unique<image_geometry::PinholeCameraModel>();
        cam_model->fromCameraInfo(cam_info);
        return cam_model;
    }

    bool _is_armed = false;
    bool _takeoff_completed = false;

    std::unique_ptr<tf2_ros::Buffer> _tf_buf;
    std::unique_ptr<tf2_ros::TransformListener> _tf_listener;
    std::unique_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;

    rclcpp::Node::SharedPtr _nh;

    image_transport::ImageTransport _it;
    image_transport::Subscriber _image_sub;

    sensor_msgs::msg::Image::ConstSharedPtr _current_image;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MissionNode>());
    rclcpp::shutdown();
    return 0;
}
