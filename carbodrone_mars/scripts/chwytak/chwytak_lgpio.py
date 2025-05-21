import rclpy
from rclpy.node import Node
import rclpy.parameter
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String
import time
from std_srvs.srv import SetBool
import lgpio


class GripperNode(Node):

    def __init__(self):
        super().__init__("gripper")

        self.declare_parameter("state", "open")
        self.declare_parameter("pin_list", [12, 18, 19, 23])
        self.declare_parameter("pose_open_list", [90, 0, 0, 0])
        self.declare_parameter("pose_close_list", [0, 90, 90, 90])

        self.service = self.create_service(
            SetBool, "change_gripper_state", self.handle_state_change
        )

        try:
            self.state: str = self.get_parameter("state").value
            self.old_state: str = "close" if self.state == "open" else "open"
            self.pin_list: list[int] = self.get_parameter("pin_list").value

            # Degrees to duty cycle [0-180] -> [2-12]
            self.pose_open_list: list[int] = [
                2.5 + degree / 18
                for degree in self.get_parameter("pose_open_list").value
            ]
            self.pose_close_list: list[int] = [
                2.5 + degree / 18
                for degree in self.get_parameter("pose_close_list").value
            ]

            self.chip = lgpio.gpiochip_open(0)
            for pin in self.pin_list:
                lgpio.gpio_claim_output(self.chip, pin)

        except Exception as e:
            self.get_logger().info(f"Gripper Node init: {e}")

        self.add_on_set_parameters_callback(self.param_callback)

        self.publisher_ = self.create_publisher(String, "gripper", 10)
        timer_period = 0.3  # seconds
        self.timer = self.create_timer(timer_period, self.servo_callback_simultaneously)

    def param_callback(self, params: list[rclpy.parameter.Parameter]):
        for param in params:
            if param.name == "state":
                self.old_state = self.state
                self.state = (
                    param.value if param.value in ["open", "close"] else "close"
                )
                self.get_logger().info(self.state)
        return SetParametersResult(successful=True)

    def handle_state_change(self, request, response):
        if request.data:
            self.state = "open"
        else:
            self.state = "close"
        response.success = True
        return response

    def servo_callback(self):
        try:
            scale = 10
            for pose_open, pose_close, pin in zip(
                [int(pose * scale) for pose in self.pose_open_list],
                [int(pose_2 * scale) for pose_2 in self.pose_close_list],
                self.pin_list,
            ):
                step_up = 1
                step_down = -1
                if pose_open > pose_close:
                    pose_open, pose_close, step_up, step_down = (
                        pose_close,
                        pose_open,
                        step_down,
                        step_up,
                    )

                if self.state == "close" and self.old_state == "open":
                    for duty in range(pose_open, pose_close, step_up):
                        # print(pin, duty / scale)
                        lgpio.tx_pwm(self.chip, pin, 50, duty / scale)
                        time.sleep(0.001)

                elif self.state == "open" and self.old_state == "close":
                    for duty in range(pose_close, pose_open, step_down):
                        # print(pin, duty / scale)
                        lgpio.tx_pwm(self.chip, pin, 50, duty / scale)
                        time.sleep(0.001)
            else:
                self.old_state = self.state

        except Exception as e:
            self.get_logger().info(f"Gripper Node timer callback: {e}")

    def servo_callback_simultaneously(self):
        scale = 10
        try:
            servos_number = [1 for _ in self.pin_list]
            current_pose = [0 for _ in self.pin_list]
            destiny_pose = [0 for _ in self.pin_list]
            direction = [0 for _ in self.pin_list]

            for idx, (pose_open, pose_close) in enumerate(
                zip(
                    self.pose_open_list,
                    self.pose_close_list,
                )
            ):
                current_pose[idx] = (
                    pose_open if self.old_state == "open" else pose_close
                )
                destiny_pose[idx] = pose_open if self.state == "open" else pose_close

                direction[idx] = 1 if destiny_pose[idx] >= current_pose[idx] else -1

            while sum(servos_number) != 0:
                for i in range(len(servos_number)):
                    if direction[i] == 1 and current_pose[i] >= destiny_pose[i]:
                        servos_number[i] = 0
                    elif direction[i] == -1 and current_pose[i] <= destiny_pose[i]:
                        servos_number[i] = 0
                    if servos_number[i] == 1:
                        current_pose[i] += direction[i] / scale

                    time.sleep(0.001)
                    # print(self.pin_list[i], current_pose[i], destiny_pose[i])
                    lgpio.tx_pwm(self.chip, self.pin_list[i], 50, current_pose[i])

            else:
                self.old_state = self.state

        except Exception as e:
            self.get_logger().info(f"Gripper Node timer callback: {e}")

    def __del__(self):
        self.get_logger().info(f"Cleaning up GPIO")
        lgpio.gpiochip_close(self.chip)


def main(args=None):
    rclpy.init(args=args)

    node = GripperNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
