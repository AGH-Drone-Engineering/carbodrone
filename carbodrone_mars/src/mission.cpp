#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/mode_completed.hpp"

using namespace std::chrono_literals;

enum class MissionState
{
    INIT = 0,
    ARM,
    TAKEOFF,
    FLIGHT,

    DONE,

    DELAY,
    CONDITION,
};

static const char *MissionStateName[] = {
    "INIT",
    "ARM",
    "TAKEOFF",
    "FLIGHT",
    "DELAY",
    "CONDITION",
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
    }

    void change_state_after(MissionState new_state, int delay)
    {
        RCLCPP_INFO(get_logger(), "Delay %d ticks", delay);
        _next_state = new_state;
        _delay_counter = delay;
        _state = MissionState::DELAY;
    }

    void change_state_after_condition(MissionState new_state, std::function<bool()> condition)
    {
        RCLCPP_INFO(get_logger(), "Wait for condition");
        _next_state = new_state;
        _delay_condition = condition;
        _state = MissionState::CONDITION;
    }

    virtual void process_state(MissionState state) = 0;

private:
    void loop()
    {
        switch (_state)
        {
        case MissionState::DELAY:
            if (_delay_counter-- > 0) break;
            change_state(_next_state);
            break;

        case MissionState::CONDITION:
            if (!_delay_condition()) break;
            change_state(_next_state);
            break;

        default:
            break;
        }

        process_state(_state);
    }

    MissionState _state = MissionState::INIT;
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
        _offboard_pub = create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", rclcpp::ServicesQoS());
        _trajectory_pub = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", rclcpp::ServicesQoS());

        _mode_completed_sub = create_subscription<px4_msgs::msg::ModeCompleted>(
            "/fmu/out/mode_completed",
            rclcpp::SensorDataQoS(),
            std::bind(&MissionNode::on_mode_completed, this, std::placeholders::_1));

        _mission_altitude = declare_parameter("mission_altitude", 5.0);
    }

private:
    void process_state(MissionState state) override
    {
        switch (state)
        {
        case MissionState::INIT:
            change_state_after(MissionState::ARM, 50);
            break;

        case MissionState::ARM:
            do_arm();
            change_state_after(MissionState::TAKEOFF, 30);
            break;

        case MissionState::TAKEOFF:
            do_takeoff(_mission_altitude);
            _mode_completed = false;
            change_state_after_condition(MissionState::FLIGHT, [this](){
                return _mode_completed;
            });
            break;

        case MissionState::FLIGHT:
            change_state(MissionState::DONE);
            break;

        case MissionState::DONE:
            rclcpp::shutdown();
            break;

        default:
            break;
        }
    }

    void send_offboard_control_mode()
    {
        px4_msgs::msg::OffboardControlMode msg;
        msg.timestamp = get_clock()->now().nanoseconds() / 1000;
        _offboard_pub->publish(std::move(msg));
    }

    void send_trajectory_setpoint()
    {
        px4_msgs::msg::TrajectorySetpoint msg;
        msg.timestamp = get_clock()->now().nanoseconds() / 1000;
        _trajectory_pub->publish(std::move(msg));
    }

    void do_arm()
    {
        RCLCPP_INFO(get_logger(), "Arming");
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
        RCLCPP_INFO(get_logger(), "Takeoff to %.2f meters", altitude);
        px4_msgs::msg::VehicleCommand msg;
        msg.timestamp = get_clock()->now().nanoseconds() / 1000;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.command = msg.VEHICLE_CMD_NAV_TAKEOFF;
        msg.param5 = 47.39794428391416;
        msg.param6 = 8.546124508703029;
        msg.param7 = altitude;
        _command_pub->publish(std::move(msg));
    }

    void on_mode_completed(const px4_msgs::msg::ModeCompleted::SharedPtr)
    {
        RCLCPP_INFO(get_logger(), "Mode completed");
        _mode_completed = true;
    }

    double _mission_altitude;

    bool _mode_completed = false;
    
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr _command_pub;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr _offboard_pub;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr _trajectory_pub;

    rclcpp::Subscription<px4_msgs::msg::ModeCompleted>::SharedPtr _mode_completed_sub;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MissionNode>());
    rclcpp::shutdown();
    return 0;
}
