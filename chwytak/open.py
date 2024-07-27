import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

def main(args=None):
    rclpy.init(args=args)
    node = Node("gripper_state_client")

    client = node.create_client(SetBool, "change_gripper_state")
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("Service not available, waiting...")

    request = SetBool.Request()
    request.data = True  # Set to True for "open" state, False for "close" state

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info(f"Service call result: {future.result().success}")
    else:
        node.get_logger().error("Service call failed")

    rclpy.shutdown()

if __name__ == "__main__":
    main()
