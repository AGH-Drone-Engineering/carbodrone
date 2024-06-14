#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/mode_completed.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/vehicle_global_position.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"

using namespace std::chrono_literals;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::ModeCompleted;
using px4_msgs::msg::VehicleStatus;
using px4_msgs::msg::VehicleGlobalPosition;
using px4_msgs::msg::VehicleLocalPosition;

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

static const double MISSION_START_ALT = 10.0;
static const double FIELD_SCAN_ALT = 5.0;
static const double BARREL_AIM_ALT = 1.0;
static const double BARREL_HEIGHT = 1.0;
static const int GRABBER_DELAY = 20;

static const double GLOBAL_LAT_ACCEPTANCE = 0.001;
static const double GLOBAL_LON_ACCEPTANCE = 0.001;
static const float GLOBAL_ALT_ACCEPTANCE = 0.5;

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
                MissionState::DO_RTL,
                std::bind(&MissionNode::reached_global_position, this, MINE_WAYPOINT[0], MINE_WAYPOINT[1], MINE_WAYPOINT[2]));
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

    void do_arm()
    {
        VehicleCommand msg;
        msg.timestamp = get_clock()->now().nanoseconds() / 1000;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.command = msg.VEHICLE_CMD_COMPONENT_ARM_DISARM;
        msg.param1 = msg.ARMING_ACTION_ARM;
        _command_pub->publish(std::move(msg));
    }

    void do_takeoff(float altitude)
    {
        VehicleCommand msg;
        msg.timestamp = get_clock()->now().nanoseconds() / 1000;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.command = msg.VEHICLE_CMD_NAV_TAKEOFF;
        msg.param5 = NAN;
        msg.param6 = NAN;
        msg.param7 = altitude;
        _command_pub->publish(std::move(msg));
    }

    void do_reposition(double latitude, double longitude, float altitude)
    {
        VehicleCommand msg;
        msg.timestamp = get_clock()->now().nanoseconds() / 1000;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.command = msg.VEHICLE_CMD_DO_REPOSITION;
        msg.param1 = -1;
        msg.param5 = latitude;
        msg.param6 = longitude;
        msg.param7 = altitude;
        _command_pub->publish(std::move(msg));
    }

    void do_return_to_launch()
    {
        VehicleCommand msg;
        msg.timestamp = get_clock()->now().nanoseconds() / 1000;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.command = msg.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH;
        _command_pub->publish(std::move(msg));
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
        double dz = _global_position->alt - altitude;

        return (fabs(dx) < GLOBAL_LAT_ACCEPTANCE &&
                fabs(dy) < GLOBAL_LON_ACCEPTANCE &&
                fabs(dz) < GLOBAL_ALT_ACCEPTANCE);
    }

    bool _is_armed = false;
    bool _takeoff_completed = false;

    VehicleGlobalPosition::SharedPtr _global_position;
    VehicleLocalPosition::SharedPtr _local_position;

    std::unique_ptr<tf2_ros::Buffer> _tf_buf;
    std::unique_ptr<tf2_ros::TransformListener> _tf_listener;
    std::unique_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;

    rclcpp::Publisher<VehicleCommand>::SharedPtr _command_pub;

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
