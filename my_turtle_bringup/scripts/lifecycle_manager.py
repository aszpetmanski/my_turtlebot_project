#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
import time


class LifecycleNodeManager(Node): 
    def __init__(self):
        super().__init__("lifecycle_manager_node")

        self.declare_parameter("managed_node_name", rclpy.Parameter.Type.STRING)
        self.managed_node_name = self.get_parameter("managed_node_name").value
        service_change_state_name = "/" + self.managed_node_name + "/change_state"
        self.change_state_client = self.create_client(ChangeState, service_change_state_name)

    def change_state(self, transition: Transition):
        self.change_state_client.wait_for_service()
        
        request = ChangeState.Request()
        request.transition = transition

        future = self.change_state_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
    
    def run_managed_node_sequence(self):
        self.get_logger().info("Start initialization sequence")
        
        self.get_logger().info("Trying to switch to configure")
        transition = Transition()
        transition.id = Transition.TRANSITION_CONFIGURE
        transition.label = "configure"
        self.change_state(transition=transition)

        time.sleep(0.2)

        self.get_logger().info("Trying to switch to active")
        transition = Transition()
        transition.id = Transition.TRANSITION_ACTIVATE
        transition.label = "activate"
        self.change_state(transition=transition)

        self.get_logger().info("Initialization sequence finished")


def main(args=None):
    rclpy.init(args=args)
    node = LifecycleNodeManager() 
    node.run_managed_node_sequence()
    rclpy.shutdown()


if __name__ == "__main__":
    main()