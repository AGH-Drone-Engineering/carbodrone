#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_services_default
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import ModeCompleted
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import OffboardControlMode
import threading


class Commander(Node):
    def __init__(self):
        super().__init__('commander')
        self._command_pub = self.create_publisher(
            VehicleCommand,
            "/fmu/in/vehicle_command",
            qos_profile_services_default
        )

        self._offboard_control_mode_sub = self.create_subscription(
            OffboardControlMode,
            "/fmu/in/offboard_control_mode",
            self.on_offboard_control_mode,
            qos_profile_sensor_data
        )

        self._mode_completed_sub = self.create_subscription(
            ModeCompleted,
            "/fmu/out/mode_completed",
            self.on_mode_completed,
            qos_profile_sensor_data
        )

        self._vehicle_status_sub = self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status",
            self.on_vehicle_status,
            qos_profile_sensor_data
        )

        self.running = True

    def on_offboard_control_mode(self, msg):
        # Implement offboard control mode callback
        pass

    def on_mode_completed(self, msg):
        # Implement mode completed callback
        pass

    def on_vehicle_status(self, msg):
        # Implement vehicle status callback
        pass

    def get_keyboard_input(self):
        while self.running:
            command = input("Enter command: ")
            self.process_command(command)

    def process_command(self, command):
        command = command.strip().split()
        if command[0] == "arm":
            self.do_arm()
        elif command[0] == "takeoff":
            self.do_takeoff(float(command[1]))
        elif command[0] == "land":
            self.do_land()
        elif command[0] == "precland":
            self.do_precision_land()
        elif command[0] == "exit":
            self.running = False
        else:
            self.get_logger().info(f"Unknown command: {command}")

    def send_vehicle_command(self, command, params=None):
        msg = VehicleCommand()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.command = command

        if params:
            msg.param1 = float(params[0]) if len(params) > 0 else 0.0
            msg.param2 = float(params[1]) if len(params) > 1 else 0.0
            msg.param3 = float(params[2]) if len(params) > 2 else 0.0
            msg.param4 = float(params[3]) if len(params) > 3 else 0.0
            msg.param5 = float(params[4]) if len(params) > 4 else 0.0
            msg.param6 = float(params[5]) if len(params) > 5 else 0.0
            msg.param7 = float(params[6]) if len(params) > 6 else 0.0

        self._command_pub.publish(msg)

    def do_arm(self):
        self.get_logger().info("[CMD] Arming vehicle")
        params = [VehicleCommand.ARMING_ACTION_ARM]
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, params)

    def do_takeoff(self, altitude):
        self.get_logger().info(f"[CMD] Taking off to {altitude:.2f} meters")
        params = [0.0, 0.0, 0.0, 0.0, float('nan'), float('nan'), altitude]
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, params)

    def do_land(self):
        self.get_logger().info("[CMD] Landing")
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def do_precision_land(self):
        self.get_logger().info("[CMD] Precision landing")
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_PRECLAND)


def main(args=None):
    rclpy.init(args=args)
    commander = Commander()
    
    input_thread = threading.Thread(target=commander.get_keyboard_input)
    input_thread.start()

    try:
        rclpy.spin(commander)
    except KeyboardInterrupt:
        pass
    finally:
        commander.running = False
        input_thread.join()
        commander.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
