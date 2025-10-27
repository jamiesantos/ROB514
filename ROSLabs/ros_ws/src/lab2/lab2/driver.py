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
from geometry_msgs.msg import Twist, Point

# math stuff
from math import atan2, tanh, sqrt, pi

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion

import actionlib
import tf

from lab2.msg import NavTargetAction, NavTargetResult, NavTargetFeedback


class Lab2Driver(Node):
	def __init__(self, position_source, threshold=0.1):
		""" We have parameters this time
		@param position_source - 
		@param threshold - how close do you have to be before turning?
		"""
		# Initialize the parent class, giving it a name.  The idiom is to use the
		# super() class.
		super().__init__('driver')

		# Goal will be set later. The action server will set the goal; you don't set it directly
		self.goal = None
		self.threshold = threshold

		self.transform_listener = tf.TransformListener()

		# Make a Marker - this will hold the goal
		self.goal_marker = Marker()
		self.goal_marker.header.frame_id = goal.goal.header.frame_id
		self.goal_marker.header.stamp = self.get_clock().now().to_msg()
		self.goal_marker.id = 0
		self.goal_marker.type = Marker.SPHERE
		self.goal_marker.action = Marker.ADD
		self.goal_marker.pose.position = None   # Will be set when we get a goal
		self.goal_marker.pose.orientation.x = 0.0
		self.goal_marker.pose.orientation.y = 0.0
		self.goal_marker.pose.orientation.z = 0.0		
		self.goal_marker.pose.orientation.w = 1.0
		self.goal_marker.scale.x = 0.3
		self.goal_marker.scale.y = 0.3
		self.goal_marker.scale.z = 0.3
		self.goal_marker.color.r = 0.0
		self.goal_marker.color.g = 1.0
		self.goal_marker.color.b = 0.0
		self.goal_marker.color.a = 1.0

		# Publisher before subscriber
		self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
		# Publish the current target as a marker (so RViz can show it)
		self.target_pub = self.create_publisher(Marker, 'current_target', 1)

		# Subscriber after publisher; this is the laser scan
		self.sub = self.create_subscription(LaserScan, 'base_scan', self.scan_callback, 1)

		# Action client
		self.action_server = actionlib.SimpleActionServer('nav_target', NavTargetAction, execute_cb=self.action_callback, auto_start=False)
		self.action_server.start()

		# Timer to check if we need to turn off the Marke
		self.timer = self.create_timer(0.1, self.timer_callback)


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

	# Respond to the action request.
	def action_callback(self, goal):
		""" This gets called when an action is received by the action server (in this case, the new goal)
		@goal - this is the new goal """
		self.get_logger().info(f'Got an action request for ({goal.goal.point.x:.2f}, {goal.goal.point.y:.2f})')

		# Set the goal.
		self.goal = goal.goal

		# Build a marker for the goal point
		#   - this prints out the green dot in RViz (the current goal)
		marker = Marker()
		self.goal_marker.header.frame_id = goal.goal.header.frame_id
		self.goal_marker.header.stamp = self.get_clock().now().to_msg()
		self.goal_marker.pose.position = goal.goal.point

		# Build a result to send back
		result = NavTargetResult()
		result.success.data = True	

		self.action_server.set_succeeded(result)

	def timer_callback(self):
		# Wait until we're at the goal.  
		# Once we get there, the callback that drives the robot will set self.goal to None.
		if self.goal:
			self.target_pub.publish(self.goal_marker)
		elif self.goal_marker.pose.position:
			# turn off the marker
			self.goal_marker.action = Marker.DELETE
			self.target_pub.publish(self.goal_marker)
			self.get_logger().info(f'Action completed')

	def scan_callback(self, lidar):
		# If we have a goal, then act on it, otherwise stay still
		if self.goal:
			# Update the timestamp on the goal and figure out where it it now in the base_link frame.
			self.goal.header.stamp = self.get_clock().now().to_msg()
			target = self.transform_listener.transformPoint('base_link', self.goal)

			self.get_logger().info(f'Target: ({target.point.x:.2f}, {target.point.y:.2f})')

			# Are we close enough?  If so, then remove the goal and stop
			distance = sqrt(target.point.x ** 2 + target.point.y ** 2)

			feedback = NavTargetFeedback()
			feedback.distance.data = distance
			self.action_server.publish_feedback(feedback)

			if distance < self.threshold:
				self.goal = None
				command = Lab2Driver.zero_twist()
			else:
				command = self.get_twist((target.point.x, target.point.y), lidar)
		else:
			command = Lab2Driver.zero_twist()

		self.cmd_pub.publish(command)


	# This is the function that controls the robot.
	#
	# Inputs:
	# 	target:	a tuple with the (x, y) coordinates of the target point, in the robot's coordinate frame (base_link).
	# 			x-axis is forward, y-axis is to the left.
	# 	lidar:	a LaserScan message with the current data from the LiDAR.  Use this for obstacle avoidance.
	#           This is the same as your go and stop code
	def get_twist(self, target, lidar):
		command = Driver.zero_twist()

		# TODO:
		#  Step 1) Calculate the angle the robot has to turn to in order to point at the target
		#  Step 2) Set your speed based on how far away you are from the target, as before
		#  Step 3) Add code that veers left (or right) to avoid an obstacle in front of it
		# This sets the move forward speed (as before)
		command.linear.x = 0.1
		# This sets the angular turn speed (in radians per second)
		command.angular.z = 0.1

  # YOUR CODE HERE

		return command

if __name__ == '__main__':
	rospy.init_node('driver', argv=sys.argv)

	driver = Driver('odom')

	rospy.spin()
