#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "Eigen/Geometry"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "px4_ros_com/frame_transforms.h"

#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/mode_completed.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/vehicle_global_position.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/goto_setpoint.hpp"

using namespace std::chrono_literals;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::ModeCompleted;
using px4_msgs::msg::VehicleStatus;
using px4_msgs::msg::VehicleGlobalPosition;
using px4_msgs::msg::VehicleLocalPosition;
using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::GotoSetpoint;
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
    X(DO_MINE_SCAN_GOTO)   \
    X(DO_MINE_SCAN_REBASE) \
    \
    X(DO_FIELD_SCAN_GOTO)  \
    X(DO_FIELD_SCAN_REBASE)\
    X(DO_FIELD_SCAN_DETECT)\
    \
    X(DO_BALL_PICKUP_LAND) \
    X(DO_BALL_PICKUP_GRAB) \
    \
    X(DO_TAKEOFF_ARM)      \
    \
    X(DO_BARREL_GOTO)      \
    X(DO_BARREL_AIM)       \
    X(DO_BARREL_DROP)      \
    X(DO_BARREL_CLEAR)     \
    \
    X(DO_RTL)              \
    \
    X(DONE)                \
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

static const double BARREL_WAYPOINT[3] = {47.398132805733496, 8.54616487844852, 10.0};

static const double MINE_WAYPOINT[3] = {47.39796990342122, 8.546164727922147, 15.0};

static const double FIELD_WAYPOINTS[9][2] = {
    {47.39794428391416,
    8.546124508703029},

    {47.39794345305943,
    8.546163475080547},

    {47.397944639996325,
    8.546205423551793},


    {47.39797073863487,
    8.546205348996033},

    {47.39796990342122,
    8.546164727922147},

    {47.39796978094731,
    8.546123857278653},


    {47.39799703247861,
    8.546121949343245},

    {47.397997851411425,
    8.546163149292989},

    {47.39799618218166,
    8.546203601239528}
};

static const double FIELD_COORDINATES[9][2] = {
    {3, 9},
    {6, 9},
    {9, 9},
    {9, 6},
    {6, 6},
    {3, 6},
    {3, 3},
    {6, 3},
    {9, 3}
};

static constexpr double MISSION_START_ALT = 10.0;
static constexpr double FIELD_SCAN_ALT = 3.0;
static constexpr double BARREL_AIM_ALT = 1.0;
static constexpr double BARREL_HEIGHT = 1.0;
static constexpr int GRABBER_DELAY = 20;
static constexpr int NUM_FIELDS = 9;

static constexpr double GLOBAL_LAT_ACCEPTANCE = 0.001;
static constexpr double GLOBAL_LON_ACCEPTANCE = 0.001;
static constexpr float GLOBAL_ALT_ACCEPTANCE = 0.5;

static constexpr double LOCAL_XY_ACCEPTANCE = 0.1;
static constexpr double LOCAL_Z_ACCEPTANCE = 0.1;

static constexpr double VEL_ACCEPANCE = 0.5;

class MissionNode : public StateMachineNode
{
public:
    MissionNode()
        : StateMachineNode("mission_node")
    {
        _tf_buf = std::make_unique<tf2_ros::Buffer>(get_clock());
        _tf_listener = std::make_unique<tf2_ros::TransformListener>(*_tf_buf);
        _tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        _command_pub = create_publisher<VehicleCommand>(
            "/fmu/in/vehicle_command", rclcpp::ServicesQoS());

        _offboard_control_mode_pub = create_publisher<OffboardControlMode>(
            "/fmu/in/offboard_control_mode", rclcpp::SensorDataQoS());

        _goto_setpoint_pub = create_publisher<GotoSetpoint>(
            "/fmu/in/goto_setpoint", rclcpp::SensorDataQoS());

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

        _offboard_timer = create_wall_timer(
            100ms,
            std::bind(&MissionNode::offboard_loop, this));
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
            RCLCPP_INFO(get_logger(), "Takeoff to %.2f meters", MISSION_START_ALT);
            do_takeoff(MISSION_START_ALT);

            _takeoff_completed = false;
            change_state_after_condition(MissionState::DO_MINE_SCAN_GOTO, [this](){
                return _takeoff_completed;
            });
            break;

        case MissionState::DO_MINE_SCAN_GOTO:
            RCLCPP_INFO(get_logger(), "Going to mine scan waypoint");
            do_reposition(MINE_WAYPOINT[0], MINE_WAYPOINT[1], MINE_WAYPOINT[2]);
            change_state_after_condition(
                MissionState::DO_MINE_SCAN_REBASE,
                std::bind(&MissionNode::reached_global_position, this, MINE_WAYPOINT[0], MINE_WAYPOINT[1], MINE_WAYPOINT[2]));
            break;

        case MissionState::DO_MINE_SCAN_REBASE:
            RCLCPP_INFO(get_logger(), "Scanning mine area");
            change_state(MissionState::DO_FIELD_SCAN_GOTO);
            break;

        case MissionState::DO_FIELD_SCAN_GOTO:
            if (_next_field_to_scan >= NUM_FIELDS)
            {
                RCLCPP_INFO(get_logger(), "All fields scanned");
                change_state(MissionState::DO_RTL);
            }
            else
            {
                RCLCPP_INFO(get_logger(), "Going to field %d", _next_field_to_scan);
                double x = FIELD_COORDINATES[_next_field_to_scan][0];
                double y = FIELD_COORDINATES[_next_field_to_scan][1];
                double z = FIELD_SCAN_ALT;
                enable_goto_setpoint(x, y, z);
                change_state_after_condition(
                    MissionState::DO_FIELD_SCAN_REBASE,
                    std::bind(&MissionNode::reached_local_position, this, x, y, z));
            }
            break;

        case MissionState::DO_FIELD_SCAN_REBASE:
            RCLCPP_INFO(get_logger(), "Rebasing at field %d", _next_field_to_scan);
            change_state(MissionState::DO_FIELD_SCAN_DETECT);
            break;

        case MissionState::DO_FIELD_SCAN_DETECT:
            RCLCPP_INFO(get_logger(), "Detecting balls at field %d", _next_field_to_scan);
            do_hold_mode();
            change_state(MissionState::DO_BALL_PICKUP_LAND);
            break;

        case MissionState::DO_BALL_PICKUP_LAND:
            RCLCPP_INFO(get_logger(), "Landing at field %d", _next_field_to_scan);
            disable_goto_setpoint();
            do_precision_land();
            change_state(MissionState::DONE);
            break;

        case MissionState::DO_RTL:
            RCLCPP_INFO(get_logger(), "Returning to launch");
            do_return_to_launch();
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
            GotoSetpoint msg;
            msg.timestamp = get_clock()->now().nanoseconds() / 1000;
            auto v = enu_to_ned_local_frame(Vector3d(_goto_setpoint_x, _goto_setpoint_y, _goto_setpoint_z + get_ground_z()));
            msg.position[0] = v.x();
            msg.position[1] = v.y();
            msg.position[2] = v.z();
            msg.flag_control_heading = true;
            _goto_setpoint_pub->publish(std::move(msg));
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

    double get_ground_z()
    {
        auto t = _tf_buf->lookupTransform("odom", "ground", tf2::TimePointZero, 50ms);
        return t.transform.translation.z;
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
    rclcpp::Publisher<GotoSetpoint>::SharedPtr _goto_setpoint_pub;

    rclcpp::Subscription<ModeCompleted>::SharedPtr _mode_completed_sub;
    rclcpp::Subscription<VehicleStatus>::SharedPtr _vehicle_status_sub;
    rclcpp::Subscription<VehicleGlobalPosition>::SharedPtr _global_position_sub;
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr _local_position_sub;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MissionNode>());
    rclcpp::shutdown();
    return 0;
}
