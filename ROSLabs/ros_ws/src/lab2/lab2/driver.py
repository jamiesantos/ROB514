#!/usr/bin/env python3

# Bill Smart, smartw@oregonstate.edu
#
# driver.py
# Drive the robot towards a goal, going around an object


# Every Python node in ROS2 should include these lines.  rclpy is the basic Python
# ROS2 stuff, and Node is the class we're going to use to set up the node.
import rclpy
from rclpy.node import Node

# Velocity commands are given with Twist messages, from geometry_msgs
from geometry_msgs.msg import Twist, PoseStamped

# math stuff
from math import atan2, tanh, sqrt, pi, fabs, cos, sin
import numpy as np

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PointStamped
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from nav_targets.action import NavTarget
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_geometry_msgs import do_transform_point
from rclpy.executors import MultiThreadedExecutor
import time


class Lab2Driver(Node):
	def __init__(self, threshold=0.4 + 0.2):
		""" We have parameters this time
		@param threshold - how close do you have to be before saying you're at the goal? Set to width of box plus width of robot
		"""
		# Initialize the parent class, giving it a name.  The idiom is to use the
		# super() class.
		super().__init__('driver')

		# Goal will be set later. The action server will set the goal; you don't set it directly
		self.goal = None
		# A controllable parameter for how close you have to be to the goal to say "I'm there"
		self.threshold = threshold

		# Make a Marker - this will hold the target goal when the action client (send points) publishes one
		self.goal_marker = None

		# Publisher before subscriber
		self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
		# Publish the current target as a marker (so RViz can show it)
		self.target_pub = self.create_publisher(Marker, 'current_target', 1)

		# Subscriber after publisher; this is the laser scan
		self.sub = self.create_subscription(LaserScan, 'base_scan', self.scan_callback, 1)

		# Create a buffer to put the data in
		self.tf_buffer = Buffer()
        
		self.transform_listener = TransformListener(self.tf_buffer, self)

		# Action client for passing "target" messages/state around
		# An action has a goal, feedback, and a result. This class (the driver) will have the action server side, and be
		#   responsible for sending feed back and result
		# The SendPoints class will have the action client - it will send the goals and cancel the goal when 
		#    distance is close enough
		self.action_server = ActionServer(node=self,
									action_type=NavTarget,
									action_name="nav_target",
									callback_group=ReentrantCallbackGroup(),
									goal_callback=self.goal_callback,
									cancel_callback=self.cancel_callback,
									execute_callback=self.action_callback)

		# Keep the current distance and target
		self.target = (0, 0)
		self.distance = 1e30

		self.last_forward = 0.0
		self.last_turn = 0.0

		# Timer to make sure we publish the target marker
		self.marker_timer = self.create_timer(1.0, self._marker_callback)

	@classmethod
	def zero_twist(cls):
		"""This is a helper class method to create and zero-out a twist"""
		command = Twist()
		command.linear.x = 0.0
		command.linear.y = 0.0
		command.linear.z = 0.0
		command.angular.x = 0.0
		command.angular.y = 0.0
		command.angular.z = 0.0

		return command

	def _marker_callback(self):
		"""Publishes the points in the list and links them up so they'll show up in RViz"""
		# If we have a marker from before, get rid of it
		if self.goal_marker:
			self.goal_marker.action = Marker.DELETE
			self.target_pub.publish(self.goal_marker)
			self.goal_marker = None
			self.get_logger().info(f"Had an existing marker; removing")

		if not self.goal:
			return
		
		# Build a marker for the goal point
		#   - this prints out the green dot in RViz (the current goal)
		# Make a Marker - this will hold the goal
		self.goal_marker = Marker()
		self.goal_marker.header.frame_id = self.goal.header.frame_id
		self.goal_marker.header.stamp = self.get_clock().now().to_msg()
		self.goal_marker.id = 0
		self.goal_marker.type = Marker.SPHERE
		self.goal_marker.action = Marker.ADD
		self.goal_marker.pose.position = self.goal.point
		self.goal_marker.scale.x = 0.3
		self.goal_marker.scale.y = 0.3
		self.goal_marker.scale.z = 0.3
		self.goal_marker.color.r = 0.0
		self.goal_marker.color.g = 1.0
		self.goal_marker.color.b = 0.0
		self.goal_marker.color.a = 1.0

		# Publish the marker
		self.target_pub.publish(self.goal_marker)
		self.get_logger().info(f"Making new marker and canceling timer")
		self.marker_timer.cancel()

	def goal_callback(self, goal_request):
		"""Accept a request for a new goal"""
		self.get_logger().info("Got a goal request")

		# Timer to make sure we publish the new target
		self.marker_timer.reset()

		return GoalResponse.ACCEPT
	
	def cancel_callback(self, goal_handle):
		"""Accept or reject a client request to cancel an action."""
		self.get_logger().info('Received cancel request')
		return CancelResponse.ACCEPT
	
	# Respond to the action request.
	def action_callback(self, goal_handle):
		""" This gets called when an action is received by the action server (in this case, the new goal)
		@goal - this is the new goal """
		self.get_logger().info(f'Got an action request... {goal_handle.request.goal.point}')
	
		# Set the new goal.
		self.goal = PointStamped()
		self.goal.header = goal_handle.request.goal.header
		self.goal.point = goal_handle.request.goal.point
		
		while self.distance > self.threshold:
			self.get_logger().info(f"looping")
			feedback = NavTarget.Feedback()
			feedback.distance.data = self.distance
			
			# publish feedback
			goal_handle.publish_feedback(feedback)

			# sleep so we can process the next scan
			time.sleep(1)

		self.get_logger().info(f"Completed goal {self.distance}")
		goal_handle.succeed()

		# Build a result to send back
		result = NavTarget.Result()
		self.get_logger().info(f"result {result}")
		result.success.data = True	
		return result

	def set_distance_to_goal(self):
		if self.goal:
			# This gets the target in the ROBOT'S coordinate frame			
			transform = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
			target = do_transform_point(self.goal, transform)
			self.target = (target.point.x, target.point.y)
			euler_ang = -atan2(2 * transform.transform.rotation.z * transform.transform.rotation.w,
			                   1.0 - 2 * transform.transform.rotation.z * transform.transform.rotation.z)
			x = self.goal.point.x - transform.transform.translation.x
			y = self.goal.point.y - transform.transform.translation.y
			self.get_logger().info(f"x y {x} {y}")
			rot_x = x * cos(euler_ang) - y * sin(euler_ang)
			rot_y = x * sin(euler_ang) + y * cos(euler_ang)
			self.target = (rot_x, rot_y)
			self.get_logger().info(f"Transform {transform.transform.translation} {euler_ang} goal {self.goal.point.x}, {self.goal.point.y}")
			
			self.get_logger().info(f'Target relative to robot: ({self.target[0]:.2f}, {self.target[1]:.2f})')

			# How close are we to the goal? Remember that the robot's location
			#  in it's own coordinate frame is at 0,0
			self.distance = sqrt(self.target[0] ** 2 + self.target[1] ** 2)
			return self.distance, self.target
		
		self.distance = 100.0
		self.target = (100, 100)

		self.get_logger().info(f'No target to get distance to')
		return 100.0, (100.0, 100.0)

	def scan_callback(self, lidar):
		self.get_logger().info("In scan callback")
		# If we have a goal, then act on it, otherwise stay still
		if self.goal:
			self.set_distance_to_goal()

			# Are we close enough?  If so, then remove the goal and stop moving
			#  The actual "we finished the goal" will happen in action_callback
			if self.distance < self.threshold:
				self.goal = None
				command = Lab2Driver.zero_twist()
			else:
				# Call the method to actually calculate the twist
				command = self.get_twist(lidar)
		else:
			command = Lab2Driver.zero_twist()
			self.get_logger().info("No goal, sitting still")

		# Publish the new twist
		self.cmd_pub.publish(command)

	def get_obstacle(self, scan):
		""" check if an obstacle"""
  # YOUR CODE HERE

	def get_twist(self, lidar):
		"""This is the method that calculate the twist
		@param target - a tuple with the (x,y) coordinates of the target point, in the robot's coordinate frame (base_link).
		    x_axis is forward, y_axis is to the left
		@param lidar - a LaserScan message with the current data from the LiDAR.  Use this for obstacle avoidance. 
		    This is the same as your go and stop code
		@return a twist command"""
		command = Lab2Driver.zero_twist()

		# GUIDE:
		#  Step 1) Calculate the angle the robot has to turn to in order to point at the target
		#  Step 2) Set your speed based on how far away you are from the target, as before
		#  Step 3) Add code that veers left (or right) to avoid an obstacle in front of it
		# Reminder: command.linear.x = 0.1     sets the forward speed to 0.1
		#           command.angular.z = 0.1   sets the angular speed
		# Reminder 2: target is in self.target and distance is in self.distance

  # YOUR CODE HERE

		self.get_logger().info(f"Setting twist forward {command.linear.x} angle {command.angular.z}")
		return command

# The idiom in ROS2 is to use a function to do all of the setup and work.  This
# function is referenced in the setup.py file as the entry point of the node when
# we're running the node with ros2 run.  The function should have one argument, for
# passing command line arguments, and it should default to None.
def main(args=None):
	# Initialize rclpy.  We should do this every time.
	rclpy.init(args=args)

	# Make a node class.  The idiom in ROS2 is to encapsulte everything in a class
	# that derives from Node.
	driver = Lab2Driver()

	# Multi-threaded execution
	executor = MultiThreadedExecutor()
	executor.add_node(driver)
	executor.spin()

	# Make sure we shutdown everything cleanly.  This should happen, even if we don't
	# include this line, but you should do it anyway.
	rclpy.shutdown()
	

# If we run the node as a script, then we're going to start here.
if __name__ == '__main__':
	# The idiom in ROS2 is to set up a main() function and to call it from the entry
	# point of the script.
	main()