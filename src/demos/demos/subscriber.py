"""subsciber.py

   Subscribe to all GUI topics.  You probably only want a subset.

   Node:       /subscriber
   Subscribe:  /boolean                 std_msgs.msg.Bool
               /float                   std_msgs.msg.Float64
               /point                   geometry_msgs.msg.PointStamped
               /pose                    geometry_msgs.msg.PoseStamped
"""

import rclpy

from asyncio                    import Future
from rclpy.node                 import Node

from std_msgs.msg               import Bool, Float64
from geometry_msgs.msg          import PointStamped, PoseStamped

from utils.TransformHelpers     import *


#
#   Demo Node Class
#
class DemoNode(Node):
    # Initialization.
    def __init__(self, name, future):
        # Initialize the node and store the future object (to end).
        super().__init__(name)
        self.future = future

        # Create the subscribers.
        self.create_subscription(Bool,         '/boolean', self.CB_bool,  10)
        self.create_subscription(Float64,      '/float',   self.CB_float, 10)
        self.create_subscription(PointStamped, '/point',   self.CB_point, 10)
        self.create_subscription(PoseStamped,  '/pose',    self.CB_pose,  10)

        # Initialize the local values.
        self.bool  = False
        self.float = 0.0
        self.p     = pzero()
        self.R     = Reye()

    # Shutdown
    def shutdown(self):
        # Simply shut down the node.
        self.destroy_node()


    # Callback functions.
    def CB_bool(self, msg):
        self.bool = msg.data
        self.get_logger().info(f"Received boolean {self.bool}")

    def CB_float(self, msg):
        self.float = msg.data
        self.get_logger().info(f"Received float {self.float}")

    def CB_point(self, msg):
        self.p = p_from_Point(msg.point)
        self.get_logger().info(f"Received point: \np:\n{self.p}")

    def CB_pose(self, msg):
        T = T_from_Pose(msg.pose)
        self.p = p_from_T(T)
        self.R = R_from_T(T)
        self.get_logger().info(f"Received pose: \np:\n{self.p}\nR:\n{self.R}")


#
#  Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Create a future object to signal when the node ends.
    future = Future()

    # Initialize the demo node.
    node = DemoNode('subscriber', future)

    # Spin, meaning keep running (taking care of the timer callbacks
    # and message passing), until interrupted or the node is
    # complete (as signaled by the future object).
    rclpy.spin_until_future_complete(node, future)

    # Report the reason for shutting down.
    if future.done():
        node.get_logger().info("Stopping: " + future.result())
    else:
        node.get_logger().info("Stopping: Interrupted")

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
