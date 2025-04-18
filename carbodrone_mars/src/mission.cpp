#define VISUALIZE_DETECTIONS 0
#define ESTIMATE_DISTANCE_FROM_DIAMETER 1
#define USE_GOTO_SETPOINT 0

#define USE_FIELD_DESCENT 0
#define FIELD_DESCENT_YOLO 0

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

#include "px4_ros_com/frame_transforms.h"

#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/mode_completed.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/vehicle_global_position.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/landing_target_pose.hpp"

#if USE_GOTO_SETPOINT
#include "px4_msgs/msg/goto_setpoint.hpp"
#else
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#endif

#include "mission_params.hpp"
#include "map_uploader.hpp"

#include "yolocpp.hpp"

using namespace std::chrono_literals;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::ModeCompleted;
using px4_msgs::msg::VehicleStatus;
using px4_msgs::msg::VehicleGlobalPosition;
using px4_msgs::msg::VehicleLocalPosition;
using px4_msgs::msg::OffboardControlMode;
#if USE_GOTO_SETPOINT
using px4_msgs::msg::GotoSetpoint;
#else
using px4_msgs::msg::TrajectorySetpoint;
#endif
using px4_msgs::msg::LandingTargetPose;
using px4_ros_com::frame_transforms::ned_to_enu_local_frame;
using px4_ros_com::frame_transforms::enu_to_ned_local_frame;
using px4_ros_com::frame_transforms::aircraft_to_baselink_body_frame;
using Eigen::Vector3d;

enum PX4_CUSTOM_MAIN_MODE
{
    PX4_CUSTOM_MAIN_MODE_MANUAL = 1,
    PX4_CUSTOM_MAIN_MODE_ALTCTL,
    PX4_CUSTOM_MAIN_MODE_POSCTL,
    PX4_CUSTOM_MAIN_MODE_AUTO,
    PX4_CUSTOM_MAIN_MODE_ACRO,
    PX4_CUSTOM_MAIN_MODE_OFFBOARD,
    PX4_CUSTOM_MAIN_MODE_STABILIZED,
    PX4_CUSTOM_MAIN_MODE_RATTITUDE_LEGACY,
    PX4_CUSTOM_MAIN_MODE_SIMPLE, /* unused, but reserved for future use */
    PX4_CUSTOM_MAIN_MODE_TERMINATION
};

enum PX4_CUSTOM_SUB_MODE_AUTO
{
    PX4_CUSTOM_SUB_MODE_AUTO_READY = 1,
    PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF,
    PX4_CUSTOM_SUB_MODE_AUTO_LOITER,
    PX4_CUSTOM_SUB_MODE_AUTO_MISSION,
    PX4_CUSTOM_SUB_MODE_AUTO_RTL,
    PX4_CUSTOM_SUB_MODE_AUTO_LAND,
    PX4_CUSTOM_SUB_MODE_AUTO_RESERVED_DO_NOT_USE, // was PX4_CUSTOM_SUB_MODE_AUTO_RTGS, deleted 2020-03-05
    PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET,
    PX4_CUSTOM_SUB_MODE_AUTO_PRECLAND,
    PX4_CUSTOM_SUB_MODE_AUTO_VTOL_TAKEOFF,
    PX4_CUSTOM_SUB_MODE_EXTERNAL1,
    PX4_CUSTOM_SUB_MODE_EXTERNAL2,
    PX4_CUSTOM_SUB_MODE_EXTERNAL3,
    PX4_CUSTOM_SUB_MODE_EXTERNAL4,
    PX4_CUSTOM_SUB_MODE_EXTERNAL5,
    PX4_CUSTOM_SUB_MODE_EXTERNAL6,
    PX4_CUSTOM_SUB_MODE_EXTERNAL7,
    PX4_CUSTOM_SUB_MODE_EXTERNAL8,
};

#define MISSION_STATE_LIST \
    X(INIT)                \
    \
    X(DO_TAKEOFF)          \
    \
    X(DO_FIELD_REPOSITION)       \
    X(DO_FIELD_REPOSITION_DELAY) \
    X(DO_FIELD_DESCENT)          \
    X(DO_FIELD_DESCENT_DELAY)    \
    X(DO_FIELD_DETECT)           \
    \
    X(DO_BALL_PICKUP_LAND)  \
    X(DO_BALL_PICKUP_GRAB)  \
    \
    X(DO_TAKEOFF_ARM)       \
    \
    X(DO_BARREL_REPOSITION)       \
    X(DO_BARREL_REPOSITION_DELAY) \
    X(DO_BARREL_DROP)       \
    \
    X(DO_RTL_REPOSITION)       \
    X(DO_RTL_REPOSITION_DELAY) \
    X(DO_RTL_LAND)             \
    \
    X(DONE) \
    X(NOOP)

static constexpr int FIELD_NOT_SCANNED = -1;
static constexpr int NUM_COLORS = 4;
static constexpr int PRIO_IGNORE = -100;

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
        , _yolo_banner(YOLO_BANNER_640_PATH, 640, 640, {"banner"})
        , _yolo_balls(YOLO_BALLS_640_PATH, 640, 640, {"blue", "green", "purple", "red"})
    {
        initialize_scanned_fields();

        _tf_buf = std::make_unique<tf2_ros::Buffer>(get_clock());
        _tf_listener = std::make_unique<tf2_ros::TransformListener>(*_tf_buf);
        _tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        _command_pub = create_publisher<VehicleCommand>(
            "/fmu/in/vehicle_command", rclcpp::ServicesQoS());

        _offboard_control_mode_pub = create_publisher<OffboardControlMode>(
            "/fmu/in/offboard_control_mode", rclcpp::SensorDataQoS());

#if USE_GOTO_SETPOINT
        _goto_setpoint_pub = create_publisher<GotoSetpoint>(
            "/fmu/in/goto_setpoint", rclcpp::SensorDataQoS());
#else
        _trajectory_setpoint_pub = create_publisher<TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", rclcpp::SensorDataQoS());
#endif

        _mode_completed_sub = create_subscription<ModeCompleted>(
            "/fmu/out/mode_completed",
            rclcpp::SensorDataQoS(),
            std::bind(&MissionNode::on_mode_completed, this, std::placeholders::_1));

        _vehicle_status_sub = create_subscription<VehicleStatus>(
            "/fmu/out/vehicle_status",
            rclcpp::SensorDataQoS(),
            std::bind(&MissionNode::on_vehicle_status, this, std::placeholders::_1));

        _global_position_sub = create_subscription<VehicleGlobalPosition>(
            "/fmu/out/vehicle_global_position",
            rclcpp::SensorDataQoS(),
            std::bind(&MissionNode::on_global_position, this, std::placeholders::_1));

        _local_position_sub = create_subscription<VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position",
            rclcpp::SensorDataQoS(),
            std::bind(&MissionNode::on_local_position, this, std::placeholders::_1));

        _target_pub = create_publisher<LandingTargetPose>(
            "/fmu/in/landing_target_pose", rclcpp::SensorDataQoS());

        _grabber_client = create_client<std_srvs::srv::SetBool>("change_gripper_state");

        _offboard_timer = create_wall_timer(
            100ms,
            std::bind(&MissionNode::offboard_loop, this));

        _map_uploader_timer = create_wall_timer(
            MAP_UPLOADER_DELAY_MS * 1ms,
            std::bind(&MissionNode::map_uploader_loop, this));
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
            if (INITIALIZE_LANDING_PAD_AFTER_MISSION_START)
            {
                RCLCPP_INFO(get_logger(), "Initializing landing pad at %.6f, %.6f", _global_position->lat, _global_position->lon);
                LANDING_PAD_WAYPOINT[0] = _global_position->lat;
                LANDING_PAD_WAYPOINT[1] = _global_position->lon;
            }
            else
            {
                RCLCPP_INFO(get_logger(), "Using hardcoded landing pad at %.6f, %.6f", LANDING_PAD_WAYPOINT[0], LANDING_PAD_WAYPOINT[1]);
            }

            do_takeoff(MISSION_START_ALT);

            _takeoff_completed = false;
            change_state_after_condition(MissionState::DO_FIELD_REPOSITION, [this](){
                return _takeoff_completed;
            });
            break;

        case MissionState::DO_FIELD_REPOSITION:
            _next_field_to_scan = decide_next_field_to_scan();
            if (_next_field_to_scan >= NUM_FIELDS)
            {
                RCLCPP_INFO(get_logger(), "All fields scanned");
                change_state(MissionState::DO_RTL_REPOSITION);
            }
            else
            {
                RCLCPP_INFO(get_logger(), "Repositioning to field %d", _next_field_to_scan);
                do_reposition(FIELD_WAYPOINTS[_next_field_to_scan][0], FIELD_WAYPOINTS[_next_field_to_scan][1], FIELD_REPOSITION_ALT);
                change_state_after_condition(
                    MissionState::DO_FIELD_REPOSITION_DELAY,
                    std::bind(&MissionNode::reached_global_position, this, FIELD_WAYPOINTS[_next_field_to_scan][0], FIELD_WAYPOINTS[_next_field_to_scan][1], FIELD_REPOSITION_ALT));
            }
            break;

        case MissionState::DO_FIELD_REPOSITION_DELAY:
            change_state_after(
#if USE_FIELD_DESCENT
                MissionState::DO_FIELD_DESCENT,
#else
                MissionState::DO_FIELD_DETECT,
#endif
                REPOSITION_DELAY);
            break;

        case MissionState::DO_FIELD_DESCENT:
            RCLCPP_INFO(get_logger(), "Descending to field %d", _next_field_to_scan);
#if USE_FIELD_DESCENT
            enable_field_descent();
#endif
            change_state_after(
                MissionState::DO_FIELD_DETECT,
                FIELD_DESCENT_DELAY);
            break;

        case MissionState::DO_FIELD_DETECT:
            RCLCPP_INFO(get_logger(), "Detecting ball at field %d", _next_field_to_scan);
            {
                int color = detect_ball_color();
                _scanned_fields[_next_field_to_scan] = color;
                if (color == _ignored_color)
                {
                    RCLCPP_INFO(get_logger(), "Detected ignored color at field %d, skipping", _next_field_to_scan);
#if USE_FIELD_DESCENT
                    disable_field_descent();
#endif
                    change_state(MissionState::DO_FIELD_REPOSITION);
                }
                else
                {
                    RCLCPP_INFO(get_logger(), "Detected ball color %d at field %d", color, _next_field_to_scan);
                    if (should_pickup_ball(color))
                    {
                        RCLCPP_INFO(get_logger(), "Correct color detected, landing");
                        change_state(MissionState::DO_BALL_PICKUP_LAND);
                    }
                    else
                    {
                        RCLCPP_INFO(get_logger(), "Incorrect color detected, skipping");
#if USE_FIELD_DESCENT
                        disable_field_descent();
#endif
                        change_state(MissionState::DO_FIELD_REPOSITION);
                    }
                }
            }
            break;

        case MissionState::DO_BALL_PICKUP_LAND:
            RCLCPP_INFO(get_logger(), "Landing at field %d", _next_field_to_scan);
#if USE_FIELD_DESCENT
            disable_field_descent();
#endif
            enable_ball_detection();
            process_ball_detection();
            do_precision_land();
            open_grabber();
            change_state_after_condition(
                MissionState::DO_BALL_PICKUP_GRAB,
                std::bind(&MissionNode::landing_completed, this));
            break;

        case MissionState::DO_BALL_PICKUP_GRAB:
            disable_ball_detection();
            RCLCPP_INFO(get_logger(), "Grabbing ball at field %d", _next_field_to_scan);
            _field_picked_up[_next_field_to_scan] = true;
            _last_picked_up_prio = _color_prio[_scanned_fields[_next_field_to_scan]];
            close_grabber();
            change_state_after(
                MissionState::DO_TAKEOFF_ARM,
                GRABBER_DELAY);
            break;

        case MissionState::DO_TAKEOFF_ARM:
            RCLCPP_INFO(get_logger(), "Taking off [arming]");
            do_arm();
            do_takeoff(MISSION_START_ALT);
            _takeoff_completed = false;
            change_state_after_condition(MissionState::DO_BARREL_REPOSITION, [this](){
                return _takeoff_completed;
            });
            break;

        case MissionState::DO_BARREL_REPOSITION:
            RCLCPP_INFO(get_logger(), "Going to barrel");
            do_reposition(BARREL_WAYPOINT[0], BARREL_WAYPOINT[1], BARREL_WAYPOINT[2]);
            change_state_after_condition(
                MissionState::DO_BARREL_REPOSITION_DELAY,
                std::bind(&MissionNode::reached_global_position, this, BARREL_WAYPOINT[0], BARREL_WAYPOINT[1], BARREL_WAYPOINT[2]));
            break;

        case MissionState::DO_BARREL_REPOSITION_DELAY:
            change_state_after(
                MissionState::DO_BARREL_DROP,
                REPOSITION_DELAY);
            break;

        case MissionState::DO_BARREL_DROP:
            RCLCPP_INFO(get_logger(), "Dropping ball at barrel");
            open_grabber();
            change_state_after(
                MissionState::DO_FIELD_REPOSITION,
                GRABBER_DELAY);
            break;

        case MissionState::DO_RTL_REPOSITION:
            RCLCPP_INFO(get_logger(), "RTL reposition");
            do_reposition(LANDING_PAD_WAYPOINT[0], LANDING_PAD_WAYPOINT[1], LANDING_PAD_WAYPOINT[2]);
            change_state_after_condition(
                MissionState::DO_RTL_REPOSITION_DELAY,
                std::bind(&MissionNode::reached_global_position, this, LANDING_PAD_WAYPOINT[0], LANDING_PAD_WAYPOINT[1], LANDING_PAD_WAYPOINT[2]));
            break;

        case MissionState::DO_RTL_REPOSITION_DELAY:
            change_state_after(
                MissionState::DO_RTL_LAND,
                REPOSITION_DELAY);
            break;

        case MissionState::DO_RTL_LAND:
            RCLCPP_INFO(get_logger(), "RTL landing");
            do_land();
            change_state(MissionState::DONE);
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

    void offboard_loop()
    {
        send_offboard_control_mode();
        send_goto_setpoint();
    }

    void send_offboard_control_mode()
    {
        OffboardControlMode msg;
        msg.timestamp = get_clock()->now().nanoseconds() / 1000;
        msg.position = true;
        _offboard_control_mode_pub->publish(std::move(msg));
    }

    void send_goto_setpoint()
    {
        if (_goto_enabled)
        {
#if USE_GOTO_SETPOINT
            GotoSetpoint msg;
            msg.timestamp = get_clock()->now().nanoseconds() / 1000;
            auto v = enu_to_ned_local_frame(Vector3d(_goto_setpoint_x, _goto_setpoint_y, _goto_setpoint_z + get_ground_z()));
            msg.position[0] = v.x();
            msg.position[1] = v.y();
            msg.position[2] = v.z();
            msg.flag_control_heading = true;
            _goto_setpoint_pub->publish(std::move(msg));
#else
            TrajectorySetpoint msg;
            msg.timestamp = get_clock()->now().nanoseconds() / 1000;
            auto v = enu_to_ned_local_frame(Vector3d(_goto_setpoint_x, _goto_setpoint_y, _goto_setpoint_z + get_ground_z()));
            msg.position[0] = v.x();
            msg.position[1] = v.y();
            msg.position[2] = v.z();
            _trajectory_setpoint_pub->publish(std::move(msg));
#endif
        }
    }

    void enable_goto_setpoint(double x, double y, double z)
    {
        RCLCPP_INFO(get_logger(), "Enabling goto setpoint: %.2f, %.2f, %.2f", x, y, z);
        _goto_setpoint_x = x;
        _goto_setpoint_y = y;
        _goto_setpoint_z = z;
        _goto_enabled = true;
        do_offboard_mode();
    }

    void disable_goto_setpoint()
    {
        RCLCPP_INFO(get_logger(), "Disabling goto setpoint");
        _goto_enabled = false;
    }

    void send_vehicle_command(uint32_t command, float *params = nullptr)
    {
        VehicleCommand msg;
        msg.timestamp = get_clock()->now().nanoseconds() / 1000;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.command = command;

        if (params)
        {
            msg.param1 = params[0];
            msg.param2 = params[1];
            msg.param3 = params[2];
            msg.param4 = params[3];
            msg.param5 = params[4];
            msg.param6 = params[5];
            msg.param7 = params[6];
        }
        
        _command_pub->publish(std::move(msg));
    }

    void do_arm()
    {
        RCLCPP_INFO(get_logger(), "[CMD] Arming vehicle");
        float params[7] = {0};
        params[0] = VehicleCommand::ARMING_ACTION_ARM;
        send_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, params);
    }

    void do_takeoff(float altitude)
    {
        RCLCPP_INFO(get_logger(), "[CMD] Taking off to %.2f meters", altitude);
        float params[7] = {0};
        params[4] = NAN;
        params[5] = NAN;
        params[6] = altitude;
        send_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF, params);
    }

    void do_offboard_mode()
    {
        RCLCPP_INFO(get_logger(), "[CMD] Switching to offboard mode");
        float params[7] = {0};
        params[0] = 1;
        params[1] = PX4_CUSTOM_MAIN_MODE_OFFBOARD;
        send_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, params);
    }

    void do_hold_mode()
    {
        RCLCPP_INFO(get_logger(), "[CMD] Switching to hold mode");
        float params[7] = {0};
        params[0] = 1;
        params[1] = PX4_CUSTOM_MAIN_MODE_AUTO;
        params[2] = PX4_CUSTOM_SUB_MODE_AUTO_LOITER;
        send_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, params);
    }

    void do_reposition(double latitude, double longitude, float altitude)
    {
        do_hold_mode();
        RCLCPP_INFO(get_logger(), "[CMD] Repositioning to %.6f, %.6f, %.2f", latitude, longitude, altitude);
        float params[7] = {0};
        params[0] = -1;
        params[4] = latitude;
        params[5] = longitude;
        params[6] = altitude;
        send_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_REPOSITION, params);
    }

    void do_return_to_launch()
    {
        RCLCPP_INFO(get_logger(), "[CMD] Returning to launch");
        send_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH);
    }

    void do_precision_land()
    {
        RCLCPP_INFO(get_logger(), "[CMD] Precision landing");
        send_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_PRECLAND);
    }

    void do_land()
    {
        RCLCPP_INFO(get_logger(), "[CMD] Landing");
        send_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
    }

    void on_mode_completed(const ModeCompleted::SharedPtr msg)
    {
        if (msg->nav_state == VehicleStatus::NAVIGATION_STATE_AUTO_TAKEOFF)
        {
            RCLCPP_INFO(get_logger(), "Takeoff completed");
            _takeoff_completed = true;
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Mode %d completed", msg->nav_state);
        }
    }

    void on_vehicle_status(const VehicleStatus::SharedPtr msg)
    {
        bool is_armed = msg->arming_state == msg->ARMING_STATE_ARMED;
        if (is_armed != _is_armed)
        {
            RCLCPP_INFO(get_logger(), "Vehicle %s", is_armed ? "armed" : "disarmed");
        }
        _is_armed = is_armed;
    }

    void on_global_position(const VehicleGlobalPosition::SharedPtr msg)
    {
        _global_position = msg;
    }

    void on_local_position(const VehicleLocalPosition::SharedPtr msg)
    {
        _local_position = msg;
    }

    bool reached_global_position(double latitude, double longitude, float altitude)
    {
        if (!_global_position)
            return false;

        double dx = _global_position->lat - latitude;
        double dy = _global_position->lon - longitude;
        float dz = _global_position->alt - altitude;

        bool pos_accept = (
            fabs(dx) < GLOBAL_LAT_ACCEPTANCE &&
            fabs(dy) < GLOBAL_LON_ACCEPTANCE &&
            fabs(dz) < GLOBAL_ALT_ACCEPTANCE);

        auto v = aircraft_to_baselink_body_frame(Vector3d(_local_position->vx, _local_position->vy, _local_position->vz));

        bool vel_accept = (
            _local_position->v_xy_valid &&
            _local_position->v_z_valid &&
            fsqrt(v.x() * v.x() + v.y() * v.y()) < VEL_ACCEPANCE &&
            fabs(v.z()) < VEL_ACCEPANCE);

        bool accept = pos_accept && vel_accept;

        if (accept)
        {
            RCLCPP_INFO(get_logger(), "Reached global position: %.6f, %.6f, %.2f", latitude, longitude, altitude);
        }

        return accept;
    }

    bool reached_local_position(double x, double y, double z)
    {
        if (!_local_position)
            return false;

        auto p = ned_to_enu_local_frame(Vector3d(_local_position->x, _local_position->y, _local_position->z));

        double dx = p.x() - x;
        double dy = p.y() - y;
        double dz = (p.z() - get_ground_z()) - z;

        bool pos_accept = (
            fsqrt(dx * dx + dy * dy) < LOCAL_XY_ACCEPTANCE &&
            fabs(dz) < LOCAL_Z_ACCEPTANCE);

        auto v = aircraft_to_baselink_body_frame(Vector3d(_local_position->vx, _local_position->vy, _local_position->vz));

        bool vel_accept = (
            _local_position->v_xy_valid &&
            _local_position->v_z_valid &&
            fsqrt(v.x() * v.x() + v.y() * v.y()) < VEL_ACCEPANCE &&
            fabs(v.z()) < VEL_ACCEPANCE);

        bool accept = pos_accept && vel_accept;

        if (accept)
        {
            RCLCPP_INFO(get_logger(), "Reached local position: %.2f, %.2f, %.2f", x, y, z);
        }

        return accept;
    }

    bool landing_completed()
    {
        return !_is_armed;
    }

    double get_ground_z()
    {
        auto t = _tf_buf->lookupTransform("odom", "ground", tf2::TimePointZero, 100ms);
        return t.transform.translation.z;
    }

#if USE_FIELD_DESCENT
    void enable_field_descent()
    {
        auto position = ned_to_enu_local_frame(Vector3d(
            _local_position->x,
            _local_position->y,
            _local_position->z
        ));
        enable_goto_setpoint(position.x(), position.y(), position.z());
        _field_descent_enabled = true;
    }

    void disable_field_descent()
    {
        _field_descent_enabled = false;
        disable_goto_setpoint();
    }
#endif

    void initialize_scanned_fields()
    {
        _scanned_fields.fill(FIELD_NOT_SCANNED);
        _color_prio.fill(PRIO_IGNORE);
        for (int i = 0; i < NUM_COLORS - 1; i++)
        {
            _color_prio[BALL_COLOR_PICKUP_ORDER[i]] = -i;
        }
        for (int i = 0; i < NUM_COLORS; i++)
        {
            if (_color_prio[i] == PRIO_IGNORE)
            {
                _ignored_color = i;
                break;
            }
        }
    }

    int detect_ball_color()
    {
        if (USE_BALL_COLOR_OVERRIDE)
        {
            return BALL_COLOR_OVERRIDE[_next_field_to_scan];
        }

        const auto &img_in = _current_image;

        auto img_ptr = cv_bridge::toCvShare(img_in, "bgr8");
        const auto &img = img_ptr->image;

        double img_cx = img.cols / 2;
        double img_cy = img.rows / 2;
        int best_color = _ignored_color;
        double best_distance = 1e9;
        const auto detections = _yolo_balls.detect(img.data, img.cols, img.rows, img.channels());
        for (const auto &detection : detections)
        {
            double x = detection.x + detection.w / 2;
            double y = detection.y + detection.h / 2;
            double distance = std::sqrt((x - img_cx) * (x - img_cx) + (y - img_cy) * (y - img_cy));
            if (distance < best_distance)
            {
                best_distance = distance;
                best_color = detection.class_id;
            }
        }
        return best_color;
    }

    bool should_pickup_ball(int color)
    {
        return _color_prio[color] == (_last_picked_up_prio - 1);
    }

    int decide_next_field_to_scan()
    {
        for (int i = 0; i < NUM_FIELDS; i++)
        {
            if (!_field_picked_up[i] && _scanned_fields[i] != FIELD_NOT_SCANNED && should_pickup_ball(_scanned_fields[i]))
            {
                RCLCPP_INFO(get_logger(), "Going to correct known color at field %d", i);
                return i;
            }
        }
        for (int i = 0; i < NUM_FIELDS; i++)
        {
            if (_scanned_fields[i] == FIELD_NOT_SCANNED)
            {
                RCLCPP_INFO(get_logger(), "Going to unknown color at field %d", i);
                return i;
            }
        }
        return NUM_FIELDS;
    }

    void enable_ball_detection()
    {
        _ball_detection_enabled = true;
    }

    void disable_ball_detection()
    {
        _ball_detection_enabled = false;
#if VISUALIZE_DETECTIONS
        cv::destroyWindow("Detections");
#endif
    }

    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &img_in)
    {
        _current_image = img_in;

        if (_ball_detection_enabled)
        {
            process_ball_detection();
        }

#if USE_FIELD_DESCENT
        if (_field_descent_enabled)
        {
            process_field_descent();
        }
#endif
    }

#if USE_FIELD_DESCENT
    void process_field_descent()
    {
        const auto &img_in = _current_image;

        rclcpp::Time time = img_in->header.stamp;

        auto img_ptr = cv_bridge::toCvShare(img_in, "bgr8");
        const auto &img = img_ptr->image;

        double img_cx = img.cols / 2;
        double img_cy = img.rows / 2;
        double best_x;
        double best_y;
        double best_area = -1;
        double best_distance = 1e9;
#if FIELD_DESCENT_YOLO
        const auto detections = _yolo_banner.detect(img.data, img.cols, img.rows, img.channels());
#else
        std::vector<YOLOCPP::Detection> detections;
        cv::Mat tmp;
        cv::cvtColor(img, tmp, cv::COLOR_RGB2GRAY);
        cv::threshold(tmp, tmp, 200, 255, cv::THRESH_BINARY);
        cv::morphologyEx(tmp, tmp, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(15, 15)));
        cv::Mat labels, stats, centroids;
        int num_labels = cv::connectedComponentsWithStats(tmp, labels, stats, centroids);
        for (int i = 1; i < num_labels; i++)
        {
            detections.push_back({
                0,
                "banner",
                1.0f,
                stats.at<int>(i, cv::CC_STAT_LEFT),
                stats.at<int>(i, cv::CC_STAT_TOP),
                stats.at<int>(i, cv::CC_STAT_WIDTH),
                stats.at<int>(i, cv::CC_STAT_HEIGHT)
            });
        }
#endif
        for (const auto &detection : detections)
        {
            double area = detection.w * detection.h;
            double x = detection.x + detection.w / 2;
            double y = detection.y + detection.h / 2;
            double distance = std::sqrt((x - img_cx) * (x - img_cx) + (y - img_cy) * (y - img_cy));
            if (distance < best_distance)
            {
                best_x = x;
                best_y = y;
                best_area = area;
                best_distance = distance;
            }
        }

#if VISUALIZE_DETECTIONS
        cv::Mat canvas;
        cv::cvtColor(img, canvas, cv::COLOR_RGB2BGR);
        for (const auto &detection : detections)
        {
            cv::rectangle(canvas, cv::Rect(detection.x, detection.y, detection.w, detection.h), cv::Scalar(0, 255, 0), 2);
        }
        cv::resize(canvas, canvas, cv::Size(), 0.5, 0.5);
        cv::imshow("Field Descent Detections", canvas);
        cv::waitKey(1);
#endif

        if (best_area < 0)
            return;

        auto cam_model = build_camera_model(img_in);

        cv::Point2d uv_raw(best_x, best_y);
        auto uv = cam_model->rectifyPoint(uv_raw);
        auto ray = cam_model->projectPixelTo3dRay(uv);

#if ESTIMATE_DISTANCE_FROM_DIAMETER
        const double diameter_mm = 1000.0 * 1.1;
        const double diameter_pixels = 2.0 * std::sqrt(best_area / M_PI);
        const double focal_length = (cam_model->fx() + cam_model->fy()) * 0.5;
        const double dist_bottom = (diameter_mm / 1000.0) * focal_length / diameter_pixels;
#else
        double dist_bottom;
        try
        {
            auto t = _tf_buf->lookupTransform(
                "ground", img_in->header.frame_id, time, 100ms);
            dist_bottom = t.transform.translation.z;
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not get height above ground");
            return;
        }
#endif

        geometry_msgs::msg::TransformStamped cam2target;
        cam2target.header.stamp = time;
        cam2target.header.frame_id = img_in->header.frame_id;
        cam2target.child_frame_id = "landing_target";

        cam2target.transform.translation.x = ray.x * dist_bottom;
        cam2target.transform.translation.y = ray.y * dist_bottom;
        cam2target.transform.translation.z = ray.z * dist_bottom;

        _tf_broadcaster->sendTransform(std::move(cam2target));

        try
        {
            auto local2target = _tf_buf->lookupTransform(
                "odom", "landing_target", cam2target.header.stamp, 100ms);

            _goto_setpoint_x = local2target.transform.translation.x;
            _goto_setpoint_y = local2target.transform.translation.y;
            _goto_setpoint_z = local2target.transform.translation.z + FIELD_SCAN_ALT;
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not get local2target transform");
        }
    }
#endif

    void process_ball_detection()
    {
        const auto &img_in = _current_image;

        rclcpp::Time time = img_in->header.stamp;

        auto img_ptr = cv_bridge::toCvShare(img_in, "bgr8");
        const auto &img = img_ptr->image;

        double img_cx = img.cols / 2;
        double img_cy = img.rows / 2;
        double best_x;
        double best_y;
        double best_area = -1;
        double best_distance = 1e9;
        const auto detections = _yolo_balls.detect(img.data, img.cols, img.rows, img.channels());
        for (const auto &detection : detections)
        {
            double area = detection.w * detection.h;
            double x = detection.x + detection.w / 2;
            double y = detection.y + detection.h / 2;
            double distance = std::sqrt((x - img_cx) * (x - img_cx) + (y - img_cy) * (y - img_cy));
            if (distance < best_distance)
            {
                best_x = x;
                best_y = y;
                best_area = area;
                best_distance = distance;
            }
        }

#if VISUALIZE_DETECTIONS
        cv::Mat canvas;
        cv::cvtColor(img, canvas, cv::COLOR_RGB2BGR);
        for (const auto &detection : detections)
        {
            cv::rectangle(canvas, cv::Rect(detection.x, detection.y, detection.w, detection.h), cv::Scalar(0, 255, 0), 2);
        }
        cv::resize(canvas, canvas, cv::Size(), 0.5, 0.5);
        cv::imshow("Detections", canvas);
        cv::waitKey(1);
#endif

        if (best_area < 0)
            return;

        auto cam_model = build_camera_model(img_in);

        cv::Point2d uv_raw(best_x, best_y);
        auto uv = cam_model->rectifyPoint(uv_raw);
        auto ray = cam_model->projectPixelTo3dRay(uv);

#if ESTIMATE_DISTANCE_FROM_DIAMETER
        const double diameter_mm = 70.0 * 1.1;
        const double diameter_pixels = 2.0 * std::sqrt(best_area / M_PI);
        const double focal_length = (cam_model->fx() + cam_model->fy()) * 0.5;
        const double dist_bottom = (diameter_mm / 1000.0) * focal_length / diameter_pixels;
#else
        double dist_bottom;
        try
        {
            auto t = _tf_buf->lookupTransform(
                "ground", img_in->header.frame_id, time, 100ms);
            dist_bottom = t.transform.translation.z;
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not get height above ground");
            return;
        }
#endif

        geometry_msgs::msg::TransformStamped cam2target;
        cam2target.header.stamp = time;
        cam2target.header.frame_id = img_in->header.frame_id;
        cam2target.child_frame_id = "landing_target";

        cam2target.transform.translation.x = ray.x * dist_bottom;
        cam2target.transform.translation.y = ray.y * dist_bottom;
        cam2target.transform.translation.z = ray.z * dist_bottom;

        _tf_broadcaster->sendTransform(std::move(cam2target));

        try
        {
            auto local2target = _tf_buf->lookupTransform(
                "odom", "landing_target", cam2target.header.stamp, 100ms);

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
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not get local2target transform");
        }
    }

    void open_grabber()
    {
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = true;

        _grabber_client->async_send_request(request);
    }

    void close_grabber()
    {
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = false;

        _grabber_client->async_send_request(request);
    }

    void map_uploader_loop()
    {
        std::vector<MapUploader::Object> objects;
        objects.emplace_back("drone", _global_position->lat, _global_position->lon);
        objects.emplace_back("landing_pad", LANDING_PAD_WAYPOINT[0], LANDING_PAD_WAYPOINT[1]);
        objects.emplace_back("barrel", BARREL_WAYPOINT[0], BARREL_WAYPOINT[1]);
        for (int i = 0; i < NUM_FIELDS; i++)
        {
            switch (_scanned_fields[i])
            {
                case FIELD_NOT_SCANNED:
                    objects.emplace_back("unknown", FIELD_WAYPOINTS[i][0], FIELD_WAYPOINTS[i][1]);
                    break;
                case 0:
                    objects.emplace_back("blue", FIELD_WAYPOINTS[i][0], FIELD_WAYPOINTS[i][1]);
                    break;
                case 1:
                    objects.emplace_back("green", FIELD_WAYPOINTS[i][0], FIELD_WAYPOINTS[i][1]);
                    break;
                case 2:
                    objects.emplace_back("purple", FIELD_WAYPOINTS[i][0], FIELD_WAYPOINTS[i][1]);
                    break;
                case 3:
                    objects.emplace_back("red", FIELD_WAYPOINTS[i][0], FIELD_WAYPOINTS[i][1]);
                    break;
            }
        }
        _map_uploader.upload_map(objects);
    }

    std::unique_ptr<image_geometry::PinholeCameraModel> build_camera_model(const sensor_msgs::msg::Image::ConstSharedPtr &img_in)
    {
        auto img_ptr = cv_bridge::toCvShare(img_in, "bgr8");
        const auto &img = img_ptr->image;

        double img_cx = img.cols / 2;
        double img_cy = img.rows / 2;

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
    int _next_field_to_scan = 0;
    double _goto_setpoint_x = 0;
    double _goto_setpoint_y = 0;
    double _goto_setpoint_z = 0;
    bool _goto_enabled = false;

    VehicleGlobalPosition::SharedPtr _global_position;
    VehicleLocalPosition::SharedPtr _local_position;

    std::unique_ptr<tf2_ros::Buffer> _tf_buf;
    std::unique_ptr<tf2_ros::TransformListener> _tf_listener;
    std::unique_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;

    rclcpp::TimerBase::SharedPtr _offboard_timer;

    rclcpp::Publisher<VehicleCommand>::SharedPtr _command_pub;
    rclcpp::Publisher<OffboardControlMode>::SharedPtr _offboard_control_mode_pub;

#if USE_GOTO_SETPOINT
    rclcpp::Publisher<GotoSetpoint>::SharedPtr _goto_setpoint_pub;
#else
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr _trajectory_setpoint_pub;
#endif

    rclcpp::Subscription<ModeCompleted>::SharedPtr _mode_completed_sub;
    rclcpp::Subscription<VehicleStatus>::SharedPtr _vehicle_status_sub;
    rclcpp::Subscription<VehicleGlobalPosition>::SharedPtr _global_position_sub;
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr _local_position_sub;

    rclcpp::Publisher<px4_msgs::msg::LandingTargetPose>::SharedPtr _target_pub;
    
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr _grabber_client;

    rclcpp::Node::SharedPtr _nh;

    image_transport::ImageTransport _it;
    image_transport::Subscriber _image_sub;

    YOLOCPP _yolo_banner;
    YOLOCPP _yolo_balls;

    std::array<int, NUM_FIELDS> _scanned_fields;
    std::array<bool, NUM_FIELDS> _field_picked_up;
    std::array<int, NUM_COLORS> _color_prio;
    int _ignored_color;

    int _last_picked_up_prio = 1;

    bool _field_descent_enabled = false;
    bool _ball_detection_enabled = false;

    sensor_msgs::msg::Image::ConstSharedPtr _current_image;

    MapUploader _map_uploader;
    rclcpp::TimerBase::SharedPtr _map_uploader_timer;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MissionNode>());
    rclcpp::shutdown();
    return 0;
}
