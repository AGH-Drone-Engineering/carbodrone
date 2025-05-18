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
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"

#include "std_msgs/msg/bool.hpp"

#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/msg/altitude.hpp"
#include "mavros_msgs/msg/waypoint.hpp"
#include "mavros_msgs/msg/command_code.hpp"
#include "mavros_msgs/msg/waypoint_list.hpp"
#include "mavros_msgs/srv/command_tol.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/waypoint_push.hpp"
#include "mavros_msgs/srv/command_long.hpp"

#include "geo.hpp"
#include "mission_params.hpp"
#include "mission_planner.hpp"

using namespace std::chrono_literals;
using Eigen::Vector3d;

#define MISSION_STATE_LIST \
    X(INIT)                \
    \
    X(DO_SEND_MISSION)        \
    X(DO_SEND_MISSION_DELAY)  \
    X(DO_EXECUTE_MISSION)     \
    X(DO_EXECUTE_MISSION_FUT) \
    X(DO_MONITOR_MISSION)     \
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
    {
        _mavros_state_sub = create_subscription<mavros_msgs::msg::State>(
            "/mavros/state",
            10,
            std::bind(&MissionNode::mavros_state_callback, this, std::placeholders::_1)
        );

        _mavros_global_position_global_sub = create_subscription<sensor_msgs::msg::NavSatFix>(
            "/mavros/global_position/global",
            rclcpp::SensorDataQoS(),
            std::bind(&MissionNode::mavros_global_position_global_callback, this, std::placeholders::_1)
        );

        _mavros_mission_waypoints_sub = create_subscription<mavros_msgs::msg::WaypointList>(
            "/mavros/mission/waypoints",
            10,
            std::bind(&MissionNode::mavros_mission_waypoints_callback, this, std::placeholders::_1)
        );

        _enable_recording_pub = create_publisher<std_msgs::msg::Bool>("enable_recording", 10);

        _mavros_command_takeoff_srv = create_client<mavros_msgs::srv::CommandTOL>(
            "/mavros/cmd/takeoff");

        _mavros_command_land_srv = create_client<mavros_msgs::srv::CommandTOL>(
            "/mavros/cmd/land");

        _mavros_set_mode_srv = create_client<mavros_msgs::srv::SetMode>(
            "/mavros/set_mode");

        _mavros_mission_push_srv = create_client<mavros_msgs::srv::WaypointPush>(
            "/mavros/mission/push");

        _mavros_command_srv = create_client<mavros_msgs::srv::CommandLong>(
            "/mavros/cmd/command");
    }

private:
    void process_state(MissionState state) override
    {
        switch (state)
        {
        case MissionState::INIT:
            change_state_after_condition(MissionState::DO_SEND_MISSION, [this](){
                return _is_armed;
            });
            break;

        case MissionState::DO_SEND_MISSION:
            do_set_global_position_rate();
            do_send_mission();
            change_state_after_condition(MissionState::DO_SEND_MISSION_DELAY, [this](){
                return _mavros_mission_push_future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
            });
            break;

        case MissionState::DO_SEND_MISSION_DELAY:
            change_state_after(MissionState::DO_EXECUTE_MISSION, 10);
            break;

        case MissionState::DO_EXECUTE_MISSION:
        {
            auto result = _mavros_mission_push_future.get();
            if (!result->success)
            {
                RCLCPP_ERROR(get_logger(), "Failed to send mission");
                change_state(MissionState::DONE);
                break;
            }
            RCLCPP_INFO(get_logger(), "Mission sent");
            do_execute_mission();
            change_state_after_condition(MissionState::DO_EXECUTE_MISSION_FUT, [this](){
                return _mavros_command_future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
            });
            break;
        }

        case MissionState::DO_EXECUTE_MISSION_FUT:
        {
            auto result = _mavros_command_future.get();
            if (!result->success)
            {
                RCLCPP_ERROR(get_logger(), "Failed to execute mission");
                change_state(MissionState::DONE);
                break;
            }
            RCLCPP_INFO(get_logger(), "Mission executed");
            change_state(MissionState::DO_MONITOR_MISSION);
            break;
        }

        case MissionState::DO_MONITOR_MISSION:
        {
            if (!_is_armed)
            {
                RCLCPP_INFO(get_logger(), "Vehicle disarmed");
                do_disable_recording();
                change_state(MissionState::DONE);
            }
            break;
        }

        case MissionState::DONE:
            if (!_is_armed)
            {
                RCLCPP_INFO(get_logger(), "Reinitializing");
                do_disable_recording();
                change_state(MissionState::INIT);
            }
            break;

        case MissionState::NOOP:
            break;

        default:
            RCLCPP_ERROR(get_logger(), "Invalid state");
            rclcpp::shutdown();
            break;
        }
    }

    void do_send_mission()
    {
        if (!_global_position_global)
        {
            RCLCPP_ERROR(get_logger(), "Send mission failed: No global position");
            return;
        }

        RCLCPP_INFO(get_logger(), "[CMD] Sending mission");
        auto request = std::make_shared<mavros_msgs::srv::WaypointPush::Request>();

        const geo::Point home(_global_position_global->latitude, _global_position_global->longitude);
        MissionPlanner planner(home);

        const geo::Point p1(50.003556, 21.106242);
        const geo::Point p2(50.003371, 21.106206);
        const geo::Point p3(50.003373, 21.106464);
        const geo::Point p4(50.003551, 21.106430);

        planner.plan(p1, p2, p3, p4, 2.0);

        auto waypoint = mavros_msgs::msg::Waypoint();
        waypoint.frame = mavros_msgs::msg::Waypoint::FRAME_GLOBAL_REL_ALT;
        waypoint.command = mavros_msgs::msg::CommandCode::NAV_WAYPOINT;
        waypoint.is_current = true;
        waypoint.autocontinue = true;
        waypoint.x_lat = home.lat();
        waypoint.y_long = home.lon();
        waypoint.z_alt = 0;
        request->waypoints.push_back(std::move(waypoint));

        waypoint = mavros_msgs::msg::Waypoint();
        waypoint.frame = mavros_msgs::msg::Waypoint::FRAME_GLOBAL_REL_ALT;
        waypoint.command = mavros_msgs::msg::CommandCode::NAV_TAKEOFF;
        waypoint.autocontinue = true;
        waypoint.x_lat = home.lat();
        waypoint.y_long = home.lon();
        waypoint.z_alt = LANDING_PAD_HOVER_ALTITUDE;
        request->waypoints.push_back(std::move(waypoint));

        waypoint = mavros_msgs::msg::Waypoint();
        waypoint.frame = mavros_msgs::msg::Waypoint::FRAME_MISSION;
        waypoint.command = mavros_msgs::msg::CommandCode::DO_CHANGE_SPEED;
        waypoint.autocontinue = true;
        waypoint.param1 = 1;
        waypoint.param2 = MISSION_GOTO_SPEED_MS;
        waypoint.param3 = -1;
        request->waypoints.push_back(std::move(waypoint));

        int wp_i = 0;
        for (const auto &point : planner.path())
        {
            waypoint = mavros_msgs::msg::Waypoint();
            waypoint.frame = mavros_msgs::msg::Waypoint::FRAME_GLOBAL_REL_ALT;
            waypoint.command = mavros_msgs::msg::CommandCode::NAV_WAYPOINT;
            waypoint.autocontinue = true;
            waypoint.x_lat = point.lat();
            waypoint.y_long = point.lon();
            waypoint.z_alt = SCAN_ALTITUDE;
            request->waypoints.push_back(std::move(waypoint));

            if (wp_i == 0)
            {
                waypoint = mavros_msgs::msg::Waypoint();
                waypoint.frame = mavros_msgs::msg::Waypoint::FRAME_MISSION;
                waypoint.command = mavros_msgs::msg::CommandCode::DO_CHANGE_SPEED;
                waypoint.autocontinue = true;
                waypoint.param1 = 1;
                waypoint.param2 = MISSION_SCAN_SPEED_MS;
                waypoint.param3 = -1;
                request->waypoints.push_back(std::move(waypoint));
            }
            wp_i++;
        }

        waypoint = mavros_msgs::msg::Waypoint();
        waypoint.frame = mavros_msgs::msg::Waypoint::FRAME_MISSION;
        waypoint.command = mavros_msgs::msg::CommandCode::DO_CHANGE_SPEED;
        waypoint.autocontinue = true;
        waypoint.param1 = 1;
        waypoint.param2 = MISSION_GOTO_SPEED_MS;
        waypoint.param3 = -1;
        request->waypoints.push_back(std::move(waypoint));

        waypoint = mavros_msgs::msg::Waypoint();
        waypoint.frame = mavros_msgs::msg::Waypoint::FRAME_GLOBAL_REL_ALT;
        waypoint.command = mavros_msgs::msg::CommandCode::NAV_WAYPOINT;
        waypoint.autocontinue = true;
        waypoint.x_lat = home.lat();
        waypoint.y_long = home.lon();
        waypoint.z_alt = LANDING_PAD_HOVER_ALTITUDE;
        request->waypoints.push_back(std::move(waypoint));

        waypoint = mavros_msgs::msg::Waypoint();
        waypoint.frame = mavros_msgs::msg::Waypoint::FRAME_GLOBAL_REL_ALT;
        waypoint.command = mavros_msgs::msg::CommandCode::NAV_LAND;
        waypoint.autocontinue = true;
        waypoint.x_lat = home.lat();
        waypoint.y_long = home.lon();
        request->waypoints.push_back(std::move(waypoint));

        auto result = _mavros_mission_push_srv->async_send_request(request);
        _mavros_mission_push_future = result.share();
    }

    void do_execute_mission()
    {
        RCLCPP_INFO(get_logger(), "[CMD] Executing mission");
        auto request = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
        request->command = mavros_msgs::msg::CommandCode::MISSION_START;
        auto result = _mavros_command_srv->async_send_request(request);
        _mavros_command_future = result.share();
    }

    void do_takeoff(float altitude)
    {
        RCLCPP_INFO(get_logger(), "[CMD] Taking off to %.2f meters", altitude);
        auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
        request->latitude = NAN;
        request->longitude = NAN;
        request->altitude = altitude;
        request->yaw = NAN;
        _mavros_command_takeoff_srv->async_send_request(request);
    }

    void do_land()
    {
        RCLCPP_INFO(get_logger(), "[CMD] Landing");
        auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
        request->latitude = NAN;
        request->longitude = NAN;
        request->yaw = NAN;
        _mavros_command_land_srv->async_send_request(request);
    }

    void do_set_global_position_rate()
    {
        RCLCPP_INFO(get_logger(), "[CMD] Setting GLOBAL_POSITION_INT rate");

        auto req  = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
        req->command = mavros_msgs::msg::CommandCode::SET_MESSAGE_INTERVAL;
        req->param1 = 33;
        req->param2 = 33333;
        req->param3 = 0;
        req->param4 = 0;
        req->param5 = 0;
        req->param6 = 0;
        req->param7 = 0;

        _mavros_command_srv->async_send_request(req);
    }

    void do_enable_recording()
    {
        RCLCPP_INFO(get_logger(), "[CMD] Enabling recording");
        std_msgs::msg::Bool msg;
        msg.data = true;
        _enable_recording_pub->publish(msg);
    }

    void do_disable_recording()
    {
        RCLCPP_INFO(get_logger(), "[CMD] Disabling recording");
        std_msgs::msg::Bool msg;
        msg.data = false;
        _enable_recording_pub->publish(msg);
    }

    void mavros_state_callback(const mavros_msgs::msg::State::ConstSharedPtr &state_in)
    {
        if (!_is_armed && state_in->armed)
        {
            RCLCPP_INFO(get_logger(), "[MAV] Vehicle armed");
        }
        else if (_is_armed && !state_in->armed)
        {
            RCLCPP_INFO(get_logger(), "[MAV] Vehicle disarmed");
        }
        _is_armed = state_in->armed;
    }

    void mavros_global_position_global_callback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr &global_position_global_in)
    {
        _global_position_global = global_position_global_in;
    }

    void mavros_mission_waypoints_callback(const mavros_msgs::msg::WaypointList::ConstSharedPtr &waypoints_in)
    {
        if (!_is_armed)
        {
            return;
        }

        int num_waypoints = waypoints_in->waypoints.size();
        if (num_waypoints < 9)
        {
            // Minimum waypoints: fake, takeoff, speed, 1goto, speed, 2goto, speed, return, land
            return;
        }

        int second_goto_idx = 5;
        int return_idx = num_waypoints - 2;

        auto second_goto = waypoints_in->waypoints[second_goto_idx];
        auto return_wp = waypoints_in->waypoints[return_idx];

        if (second_goto.is_current)
        {
            do_enable_recording();
        }
        else if (return_wp.is_current)
        {
            do_disable_recording();
        }
    }

    rclcpp::Node::SharedPtr _nh;

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr _mavros_state_sub;
    bool _is_armed = false;

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr _mavros_global_position_global_sub;
    sensor_msgs::msg::NavSatFix::ConstSharedPtr _global_position_global;

    rclcpp::Subscription<mavros_msgs::msg::WaypointList>::SharedPtr _mavros_mission_waypoints_sub;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _enable_recording_pub;

    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr      _mavros_command_takeoff_srv;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr      _mavros_command_land_srv;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr         _mavros_set_mode_srv;
    rclcpp::Client<mavros_msgs::srv::WaypointPush>::SharedPtr    _mavros_mission_push_srv;
    rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr     _mavros_command_srv;

    std::shared_future<mavros_msgs::srv::WaypointPush::Response::SharedPtr> _mavros_mission_push_future;
    std::shared_future<mavros_msgs::srv::CommandLong::Response::SharedPtr> _mavros_command_future;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MissionNode>());
    rclcpp::shutdown();
    return 0;
}
