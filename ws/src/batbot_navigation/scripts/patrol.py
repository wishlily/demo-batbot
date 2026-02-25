#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import time
import sys

def main():
    rclpy.init()
    nav = BasicNavigator()

    # --- 1. Define waypoints ---
    waypoints = []

    # Point A
    goal_a = PoseStamped()
    goal_a.header.frame_id = 'map'
    goal_a.pose.position.x = 1.0
    goal_a.pose.position.y = 0.0
    goal_a.pose.orientation.w = 1.0
    waypoints.append(goal_a)

    # Point B
    goal_b = PoseStamped()
    goal_b.header.frame_id = 'map'
    goal_b.pose.position.x = 2.4
    goal_b.pose.position.y = 0.4
    goal_b.pose.orientation.w = 1.0
    waypoints.append(goal_b)

    # Point C (Origin)
    goal_c = PoseStamped()
    goal_c.header.frame_id = 'map'
    goal_c.pose.position.x = 0.0
    goal_c.pose.position.y = 0.0
    goal_c.pose.orientation.w = 1.0
    waypoints.append(goal_c)

    # Wait for navigation system to fully activate
    nav.waitUntilNav2Active()

    print("--- Starting patrol task (Press Ctrl+C to stop) ---")

    try:
        while rclpy.ok():
            # Send multi-point follow request
            nav.followWaypoints(waypoints)

            # Monitor task status
            while not nav.isTaskComplete():
                feedback = nav.getFeedback()
                if feedback:
                    # Print current waypoint being navigated to
                    # feedback.current_waypoint is the index of the current target (starting from 0)
                    print(f'Navigating to waypoint {feedback.current_waypoint + 1}...', end='\r')

            # Check final result
            result = nav.getResult()
            if result == TaskResult.SUCCEEDED:
                print("\n[SUCCESS] Patrol round completed successfully.")
            elif result == TaskResult.CANCELED:
                print("\n[CANCELED] Patrol task has been cancelled.")
                break
            elif result == TaskResult.FAILED:
                print("\n[FAILED] Patrol task failed, retrying...")

            # Interval between rounds
            print("Starting next patrol round in 1 second...")
            time.sleep(1)

    except KeyboardInterrupt:
        # --- 2. Graceful shutdown logic ---
        print('\n\nCtrl+C detected, cancelling current navigation task...')
        nav.cancelTask()

        # Wait a little while for the cancel command to go through
        timeout = 2.0
        start_time = time.time()
        while not nav.isTaskComplete() and (time.time() - start_time) < timeout:
            time.sleep(0.1)

        print("Task cancelled, shutting down node.")

    finally:
        # Ensure that the following code will always be executed
        # Note: BasicNavigator may not require explicit shutdown in some versions,
        # but it is a good practice to include it to prevent resource leaks
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(0)

if __name__ == '__main__':
    main()