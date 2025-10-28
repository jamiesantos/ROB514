#!/usr/bin/env python3

# Bill Smart, smartw@oregonstate.edu
#
# driver.py
# Send navigation targets to the robot


# Every Python node in ROS2 should include these lines.  rclpy is the basic Python
# ROS2 stuff, and Node is the class we're going to use to set up the node.
import rclpy
from rclpy.node import Node

import numpy as np

from threading import Lock

from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter

from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.action import ActionClient
from nav_targets.action import NavTarget
from rclpy.executors import MultiThreadedExecutor
import asyncio


class SendPoints(Node):
	def __init__(self, points):
		""" Initialize way points
		@param - points, an iterable list of x,y tuples"""
		# Initialize the parent class, giving it a name.  The idiom is to use the
		# super() class.
		super().__init__('send_points')

		# A mutex to keep us safe during the list deletions.
		self.mutex = Lock()

		# An action server to send the requests to.
		self.action_client = ActionClient(node=self, action_type=NavTarget, action_name='nav_target')

		# Save the points for when we startup
		self.current_point = 0
		self.points = [p for p in points]

		self.send_goal_future = None
		self.last_distance = 1e30

		# Timer to make sure we publish the target marker
		self.start_timer = self.create_timer(1.0, self._start_action_client)

		# Publisher for the visualization
		self.marker_pub = self.create_publisher(MarkerArray, 'goal_points', 1)

	def _start_action_client(self):
		if self.current_point == 0:
			# Wait for driver to start
			self.get_logger().info("Start driver.py to get started")
			self.action_client.wait_for_server()
		
		# Cancel the timer
		self.start_timer.cancel()

		if self.current_point >= len(self.points):
			self.get_logger().info("No more points to send")
			return
			
		if self.current_point == 0:
			self.set_marker_points()

		# send the goal
		pt = self.points[self.current_point]
		self.current_point += 1

		goal = NavTarget.Goal()
		goal.goal.header.frame_id = 'odom'
		goal.goal.header.stamp = self.get_clock().now().to_msg()

		goal.goal.point.x = float(pt[0])
		goal.goal.point.y = float(pt[1])
		goal.goal.point.z = 0.0

		self.get_logger().info(f'Sending goal request... {self.current_point-1} of {len(self.points)} {pt[0], pt[1]}')

		self.send_goal_future = self.action_client.send_goal_async(goal=goal, 
														 feedback_callback=self._feedback_callback)
		self.send_goal_future.add_done_callback(self._goal_done_callback)

	def _goal_done_callback(self, future):
		if future.done():
			self.get_logger().info(f"done goal {future.result()}")
			if self.last_distance < 1.0:
				self.start_timer.reset()		
		else:
			self.get_logger().info(f"Not done goal  {future.result()}")

	def _feedback_callback(self, feedback):
		"""Every time we get a laser scan (that triggers the robot moving), send back the distance to the target as feedbackack
		@param feedback - data created by the action server - this has the distance in it (as a float)"""
		# Right now not doing anything but publishing the current distance
		self.last_distance = feedback.feedback.distance.data
		self.get_logger().info(f'Feedback: Distance: {feedback.feedback.distance.data}')

	def set_marker_points(self):
		"""Publishes the points in the list and links them up so they'll show up in RViz"""
		array = MarkerArray()

		# Lock while we make the Marker Array
		with self.mutex:
			marker = Marker()
			marker.header.frame_id = 'odom'
			marker.header.stamp = self.get_clock().now().to_msg()
			marker.id = 0
			marker.type = Marker.LINE_STRIP
			marker.action = Marker.ADD
			marker.scale.x = 0.1
			marker.scale.y = 0.1
			marker.scale.z = 0.1
			marker.color.r = 0.0
			marker.color.g = 0.0
			marker.color.b = 1.0
			marker.color.a = 1.0
			marker.points = []
			for p in self.points:
				pt = Point()
				pt.x = p[0]
				pt.y = p[1]
				pt.z = 0.0
				marker.points.append(pt)
			
			# Make the line(s) between the markers
			array.markers.append(marker)

			# Make the dots for the markers
			for indx, point in enumerate(self.points):
				marker = Marker()
				marker.header.frame_id = 'odom'
				marker.header.stamp = self.get_clock().now().to_msg()
				marker.id = indx + 1
				marker.type = Marker.SPHERE
				marker.action = Marker.ADD
				marker.pose.position.x = point[0]
				marker.pose.position.y = point[1]
				marker.pose.position.z = 0.0
				marker.pose.orientation.x = 0.0
				marker.pose.orientation.y = 0.0
				marker.pose.orientation.z = 0.0		
				marker.pose.orientation.w = 1.0
				marker.scale.x = 0.2
				marker.scale.y = 0.2
				marker.scale.z = 0.2
				marker.color.r = 0.0
				marker.color.g = 0.0
				marker.color.b = 1.0
				marker.color.a = 1.0

				array.markers.append(marker)

		# Actually publish the list
		self.marker_pub.publish(array)

# Unlike all the previous code, here we'll start up with a list of points to go to
def main(args=None):
	# Initialize rclpy.  We should do this every time.
	rclpy.init(args=args)

	# Create a list of points on a circle. Send points will turn them into markers and send goals to driver
	points = [(5 * np.cos(theta), 5 * np.sin(theta)) for theta in np.linspace(0.0, 2 * np.pi, 15)]
	send_points = SendPoints(points)

	# Multi-threaded execution
	executor = MultiThreadedExecutor()
	executor.add_node(send_points)
	executor.spin()

	# Make sure we shutdown everything cleanly.  This should happen, even if we don't
	# include this line, but you should do it anyway.
	rclpy.shutdown()
	

# If we run the node as a script, then we're going to start here.
if __name__ == '__main__':
	# The idiom in ROS2 is to set up a main() function and to call it from the entry
	# point of the script.
	main()