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
