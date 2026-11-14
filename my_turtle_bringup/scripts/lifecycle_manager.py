#!/usr/bin/env python3

"""
Manage the lifecycle of a ROS 2 node by automatically transitioning it
through the configuration and activation states during startup.
"""

import time

import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition


class LifecycleNodeManager(Node):
    def __init__(self):
        super().__init__("lifecycle_manager")

        # Get the name of the lifecycle node to manage.
        self.declare_parameter("managed_node_name", rclpy.Parameter.Type.STRING)
        self.managed_node_name = self.get_parameter("managed_node_name").value

        # Create a client for requesting lifecycle state transitions.
        service_change_state_name = f"/{self.managed_node_name}/change_state"
        self.change_state_client = self.create_client(
            ChangeState, service_change_state_name
        )

    def change_state(self, transition: Transition) -> bool:
        # Wait for the lifecycle service to become available.
        self.change_state_client.wait_for_service()

        # Send the requested transition and wait for the response.
        request = ChangeState.Request()
        request.transition = transition

        future = self.change_state_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.exception() is not None:
            self.get_logger().error(
                f"Failed to execute '{transition.label}' transition: "
                f"{future.exception()}."
            )
            return False

        response = future.result()

        if response is None or not response.success:
            self.get_logger().error(
                f"Lifecycle transition '{transition.label}' failed."
            )
            return False

        self.get_logger().info(
            f"Lifecycle transition '{transition.label}' completed successfully."
        )
        return True

    def run_managed_node_sequence(self):
        self.get_logger().info("Starting lifecycle initialization sequence.")

        # Configure the managed node before attempting activation.
        self.get_logger().info("Requesting configuration transition.")

        transition = Transition()
        transition.id = Transition.TRANSITION_CONFIGURE
        transition.label = "configure"

        if not self.change_state(transition):
            self.get_logger().error("Lifecycle initialization aborted.")
            return

        time.sleep(0.2)

        # Activate the configured node to enable normal operation.
        self.get_logger().info("Requesting activation transition.")

        transition = Transition()
        transition.id = Transition.TRANSITION_ACTIVATE
        transition.label = "activate"

        if not self.change_state(transition):
            self.get_logger().error("Lifecycle initialization aborted.")
            return

        self.get_logger().info(
            "Lifecycle initialization sequence completed successfully."
        )


def main(args=None):
    rclpy.init(args=args)

    node = LifecycleNodeManager()
    try:
        node.run_managed_node_sequence()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()