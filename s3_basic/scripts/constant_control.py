#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

# import the message type to use
from std_msgs.msg import Int64, Bool
from geometry_msgs.msg import Twist


class Heartbeat(Node):
    def __init__(self) -> None:
            # initialize base class (must happen before everything else)
        super().__init__("heartbeat")
            
            # a heartbeat counter
        self.hb_counter = 0

            # create publisher with: self.create_publisher(<msg Twisttype>, <topic>, <qos>)
        self.hb_pub = self.create_publisher(Twist, "/cmd_vel", 10)
    
        # create a timer with: self.create_timer(<second>, <callback>)
        self.hb_timer = self.create_timer(0.2, self.hb_callback)

        # create subscription with: self.create_subscription(<msg type>, <topic>, <callback>, <qos>)
        self.motor_sub = self.create_subscription(Bool, "/health/motor", self.health_callback, 10)
        self.motor_sub = self.create_subscription(Bool, "/kill", self.kill_callback, 10)
          

    def hb_callback(self) -> None:
        """
        Heartbeat callback triggered by the timer
        """
        # construct heartbeat message
        msg = Twist()  # 0 initialize everything by default
        msg.linear.x = float(self.hb_counter)  # set this to be the linear velocity
        msg.angular.x = 0 # set this to be the angular velocity
        msg.angular.y = 0 # set this to be the angular velocity
        msg.angular.z = 0 # set this to be the angular velocity

        # publish heartbeat counter
        self.hb_pub.publish(msg)

        # increment counter
        # self.hb_counter += 1

    def health_callback(self, msg: Bool) -> None:
        """
        Sensor health callback triggered by subscription
        """
        if not msg.data:
            self.get_logger().fatal("Heartbeat stopped")
            self.hb_timer.cancel()

    def kill_callback(self, msg: Bool) -> None:
        """
        Sensor health callback triggered by subscription
        """
        self.hb_timer.cancel()
        msg = Twist()  # 0 initialize everything by default
        msg.linear.x = float(0)  # set this to be the linear velocity
        msg.angular.x = float(0) # set this to be the angular velocity
        self.hb_pub.publish(msg)
        


if __name__ == "__main__":
    rclpy.init()        # initialize ROS2 context (must run before any other rclpy call)
    node = Heartbeat()  # instantiate the heartbeat node
    rclpy.spin(node)    # Use ROS2 built-in schedular for executing the node
    rclpy.shutdown()    # cleanly shutdown ROS2 context