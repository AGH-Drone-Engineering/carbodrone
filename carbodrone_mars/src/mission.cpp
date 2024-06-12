#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/mode_completed.hpp"
#include "px4_msgs/msg/vehicle_global_position.hpp"

using namespace std::chrono_literals;

enum class MissionState
{
    INIT = 0,
    ARM,
    TAKEOFF,

    DONE,

    NOOP,
};

static const char *MissionStateName[] = {
    "INIT",
    "ARM",
    "TAKEOFF",

    "DONE",

    "NOOP",
};

static const double BARREL_WAYPOINT[2] = {47.398132805733496, 8.54616487844852};

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

class StateMachineNode : public rclcpp::Node
{
protected:
    StateMachineNode(const std::string &name)
        : Node(name)
    {
        _loop_timer = create_wall_timer(100ms, std::bind(&StateMachineNode::loop, this));
    }

    void change_state(MissionState new_state)
    {
        RCLCPP_INFO(get_logger(), "State change: %s -> %s", MissionStateName[(int)_state], MissionStateName[(int)new_state]);
        _state = new_state;
        _state_delay = false;
        _state_condition = false;
    }

    void change_state_after(MissionState new_state, int delay)
    {
        RCLCPP_INFO(get_logger(), "Delay %d ticks", delay);
        _next_state = new_state;
        _delay_counter = delay;
        _state_delay = true;
        _state_condition = false;
    }

    void change_state_after_condition(MissionState new_state, std::function<bool()> condition)
    {
        RCLCPP_INFO(get_logger(), "Wait for condition");
        _next_state = new_state;
        _delay_condition = condition;
        _state_delay = false;
        _state_condition = true;
    }

    virtual void process_state(MissionState state) = 0;

private:
    void loop()
    {
        if (_state_delay)
        {
            if (_delay_counter-- > 0)
                return process_state(MissionState::NOOP);
            else change_state(_next_state);
        }
        else if (_state_condition)
        {
            if (!_delay_condition())
                return process_state(MissionState::NOOP);
            else change_state(_next_state);
        }
        process_state(_state);
    }

    MissionState _state = MissionState::INIT;
    bool _state_delay = false;
    bool _state_condition = false;
    int _delay_counter;
    std::function<bool()> _delay_condition;
    MissionState _next_state;
    rclcpp::TimerBase::SharedPtr _loop_timer;
};

class MissionNode : public StateMachineNode
{
public:
    MissionNode()
        : StateMachineNode("mission_node")
    {
        _command_pub = create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", rclcpp::ServicesQoS());

        _mode_completed_sub = create_subscription<px4_msgs::msg::ModeCompleted>(
            "/fmu/out/mode_completed",
            rclcpp::SensorDataQoS(),
            std::bind(&MissionNode::on_mode_completed, this, std::placeholders::_1));

        _global_position_sub = create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
            "/fmu/out/vehicle_global_position",
            rclcpp::SensorDataQoS(),
            std::bind(&MissionNode::on_global_position, this, std::placeholders::_1));

        _mission_altitude = declare_parameter("mission_altitude", 5.0);
    }

private:
    void process_state(MissionState state) override
    {
        switch (state)
        {
        case MissionState::INIT:
            change_state_after(MissionState::ARM, 20);
            break;

        case MissionState::ARM:
            RCLCPP_INFO(get_logger(), "Arming");
            do_arm();
            change_state_after(MissionState::TAKEOFF, 30);
            break;

        case MissionState::TAKEOFF:
            _mission_start_lat = _global_position->lat;
            _mission_start_lon = _global_position->lon;
            _mission_start_alt_ellipsoid = _global_position->alt_ellipsoid;
            RCLCPP_INFO(get_logger(), "Mission started at: %.6f, %.6f, %.2f", _mission_start_lat, _mission_start_lon, _mission_start_alt_ellipsoid);

            RCLCPP_INFO(get_logger(), "Takeoff to %.2f meters", _mission_altitude);
            do_takeoff(_mission_altitude);
            
            _mode_completed = false;
            change_state_after_condition(MissionState::DONE, [this](){
                return _mode_completed;
            });
            break;

        case MissionState::DONE:
            RCLCPP_INFO(get_logger(), "Mission completed");
            rclcpp::shutdown();
            break;

        case MissionState::NOOP:
            break;
        }
    }

    void do_arm()
    {
        px4_msgs::msg::VehicleCommand msg;
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
        px4_msgs::msg::VehicleCommand msg;
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

    void on_mode_completed(const px4_msgs::msg::ModeCompleted &msg)
    {
        RCLCPP_INFO(get_logger(), "Mode %d completed", msg.nav_state);
        _mode_completed = true;
    }

    void on_global_position(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg)
    {
        _global_position = msg;
    }

    double _mission_altitude;
    
    bool _mode_completed = false;
    
    double _mission_start_lat;
    double _mission_start_lon;
    double _mission_start_alt_ellipsoid;
    
    px4_msgs::msg::VehicleGlobalPosition::SharedPtr _global_position;

    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr _command_pub;

    rclcpp::Subscription<px4_msgs::msg::ModeCompleted>::SharedPtr _mode_completed_sub;
    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr _global_position_sub;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MissionNode>());
    rclcpp::shutdown();
    return 0;
}
