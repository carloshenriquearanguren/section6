#!/usr/bin/env python3

import numpy as np
from scipy.signal import convolve2d
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid
from asl_tb3_msgs.msg import TurtleBotState
from asl_tb3_lib.grids import StochOccupancyGrid2D

current_state = np.array([0.0, 0.0])

def explore(occupancy):
    """Returns frontier states as (N, 2) array in world coordinates."""
    window_size = 13
    w = np.ones((window_size, window_size), dtype=float)
    
    probs = occupancy.probs
    unknown = (probs < 0)
    occupied = (probs >= occupancy.thresh)
    free = (probs >= 0) & (probs < occupancy.thresh)
    
    unk_count = convolve2d(unknown.astype(float), w, mode="same", boundary="fill", fillvalue=0)
    occ_count = convolve2d(occupied.astype(float), w, mode="same", boundary="fill", fillvalue=0)
    free_count = convolve2d(free.astype(float), w, mode="same", boundary="fill", fillvalue=0)
    total_count = convolve2d(np.ones_like(probs, dtype=float), w, mode="same", boundary="fill", fillvalue=0)
    
    cond1 = unk_count >= 0.20 * total_count
    cond2 = occ_count == 0
    cond3 = free_count >= 0.30 * total_count
    
    frontier_mask = cond1 & cond2 & cond3 & free
    ys, xs = np.where(frontier_mask)
    
    if ys.size == 0:
        return np.empty((0, 2))
    
    grid_xy = np.stack([xs, ys], axis=1)
    return occupancy.grid2state(grid_xy)


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__("frontier_explorer")
        self.occupancy = None
        self.waiting = True
        self.goal_sent_time = None
        self.timeout_sec = 30.0  # Timeout if goal not reached in 30 seconds
        
        self.cmd_nav_pub = self.create_publisher(TurtleBotState, "/cmd_nav", 10)
        self.create_subscription(Bool, "/nav_success", self.nav_success_callback, 10)
        self.create_subscription(TurtleBotState, "/state", self.state_callback, 10)
        self.create_subscription(OccupancyGrid, "/map", self.map_callback, 10)
        
        # Timer to check for timeout
        self.create_timer(2.0, self.timeout_check)
    
    def state_callback(self, msg):
        global current_state
        current_state = np.array([msg.x, msg.y])
    
    def map_callback(self, msg):
        self.occupancy = StochOccupancyGrid2D(
            resolution=msg.info.resolution,
            size_xy=np.array([msg.info.width, msg.info.height]),
            origin_xy=np.array([msg.info.origin.position.x, msg.info.origin.position.y]),
            window_size=9,
            probs=msg.data,
            thresh=0.5
        )
        if self.waiting:
            self.send_goal()
    
    def nav_success_callback(self, msg):
        """Called when navigation succeeds or fails."""
        if msg.data:
            self.get_logger().info("Goal reached")
            self.waiting = True
            self.send_goal()
        else:
            self.get_logger().warn("Navigation failed, trying different frontier")
            self.waiting = True
            self.send_goal(skip_closest=True)
    
    def timeout_check(self):
        """Check if current goal has timed out."""
        if not self.waiting and self.goal_sent_time is not None:
            elapsed = self.get_clock().now().nanoseconds / 1e9 - self.goal_sent_time
            if elapsed > self.timeout_sec:
                self.get_logger().warn(f"Goal timeout after {elapsed:.1f}s, picking new frontier")
                self.waiting = True
                self.send_goal(skip_closest=True)
    
    def send_goal(self, skip_closest=False):
        if not self.occupancy:
            return
        
        frontiers = explore(self.occupancy)
        if frontiers.shape[0] == 0:
            self.get_logger().info("No frontiers - exploration complete")
            return
        
        dists = np.linalg.norm(frontiers - current_state[None, :], axis=1)
        
        if skip_closest and frontiers.shape[0] > 1:
            closest_idx = np.argmin(dists)
            mask = np.ones(len(dists), dtype=bool)
            mask[closest_idx] = False
            frontiers = frontiers[mask]
            dists = dists[mask]
            
            if frontiers.shape[0] == 0:
                self.get_logger().info("No alternative frontiers")
                return
        
        goal_xy = frontiers[np.argmin(dists)]
        
        goal = TurtleBotState()
        goal.x = float(goal_xy[0])
        goal.y = float(goal_xy[1])
        goal.theta = 0.0
        
        self.cmd_nav_pub.publish(goal)
        self.waiting = False
        self.goal_sent_time = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info(f"Sent goal: ({goal.x:.2f}, {goal.y:.2f})")


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
