#!/usr/bin/env python3
import rospy

# publishing to /cmd_vel with msg type: Twist
from geometry_msgs.msg import Twist
# subscribing to /odom with msg type: Odometry
from nav_msgs.msg import Odometry

# for finding sin() cos() 
import math

from time import sleep

# Odometry is given as a quaternion, but for the controller we'll need to find the orientaion theta by converting to euler angle
from tf.transformations import euler_from_quaternion

from geometry_msgs.msg import PoseArray

#######################################goal##########################################################
x_goals = []
y_goals = []
theta_goals = []

def task1_goals_Cb(msg):
	global x_goals, y_goals, theta_goals

	x_goals.clear()
	y_goals.clear()
	theta_goals.clear()

	for waypoint_pose in msg.poses:
		x_goals.append(waypoint_pose.position.x)
		y_goals.append(waypoint_pose.position.y)

		orientation_q = waypoint_pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		theta_goal = euler_from_quaternion (orientation_list)[2]
		theta_goals.append(theta_goal)




class HolaBot:
	def __init__(self):
		self.x = 0
		self.y = 0
		self.theta = 0
		self.roll = 0
		self.pitch = 0

		# initialize the node
		rospy.init_node('controller', anonymous=True)
		#initialize the publisher and subscriber
		self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		self.sub = rospy.Subscriber('/odom', Odometry, self.odometryCb)
		# set the rate
		self.rate = rospy.Rate(100) # 10hz


	def odometryCb(self,msg: Odometry):
		# get the position and orientation from the odometry message
		self.x = round(msg.pose.pose.position.x,4)
		self.y = round(msg.pose.pose.position.y,4)
		orientation_q = msg.pose.pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		self.theta = euler_from_quaternion (orientation_list)[2]
		# print(self.x, self.y, self.theta)


	def euclidean_distance(self, goal_x, goal_y):
		return math.sqrt(pow((goal_x - self.x), 2) + pow((goal_y - self.y), 2))

	def steering_angle(self, goal_x, goal_y):
		return math.atan2(goal_y - self.y, goal_x - self.x)

	def angular_vel(self, goal_theta, constant=6):
		return constant * (goal_theta - self.theta)

	def linear_vel(self, goal_x,goal_y, constant=4):
		return constant * self.euclidean_distance(goal_x,goal_y)

	def rotate(self,theta_goal):
		"""Rotates the turtle to the goal."""
		if theta_goal > math.pi:
			theta_goal -= 2*math.pi
		print("theta_goal: ", theta_goal)
		# Please, insert a number slightly greater than 0 (e.g. 0.01).
		angle_tolerance = 0.0001


		vel_msg = Twist()

		while abs(theta_goal - self.theta) >= angle_tolerance:

			# Porportional controller.
			# https://en.wikipedia.org/wiki/Proportional_control

			# Linear velocity in the x-axis.
			vel_msg.linear.x = 0
			vel_msg.linear.y = 0
			vel_msg.linear.z = 0

			# Angular velocity in the z-axis.
			vel_msg.angular.x = 0
			vel_msg.angular.y = 0
			vel_msg.angular.z = self.angular_vel(theta_goal)

			# Publishing our vel_msg
			self.pub.publish(vel_msg)

			# Publish at the desired rate.
			self.rate.sleep()

		# Stopping our robot after the movement is over.
		vel_msg.linear.x = 0
		vel_msg.angular.z = 0
		self.pub.publish(vel_msg)


	def move2goal(self,goal_x,goal_y,theta_goal):
		"""Moves the turtle to the goal."""


        # Please, insert a number slightly greater than 0 (e.g. 0.01).
		distance_tolerance = 0.09
		print("goal_x: ", goal_x, "\ngoal_y: ", goal_y)
		self.rotate(self.steering_angle(goal_x,goal_y))

		vel_msg = Twist()

		while self.euclidean_distance(goal_x,goal_y) >= distance_tolerance:

            # Porportional controller.
            # https://en.wikipedia.org/wiki/Proportional_control

            # Linear velocity in the x-axis.
			vel_msg.linear.x = self.linear_vel(goal_x,goal_y)
			vel_msg.linear.y = 0
			vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
			vel_msg.angular.x = 0
			vel_msg.angular.y = 0
			vel_msg.angular.z = 0#self.angular_vel(self.steering_angle(goal_x,goal_y), constant=.5)
            # Publishing our vel_msg
			self.pub.publish(vel_msg)

            # Publish at the desired rate.
			self.rate.sleep()

        # Stopping our robot after the movement is over.
		vel_msg.linear.x = 0
		vel_msg.angular.z = 0
		self.pub.publish(vel_msg)

		self.rotate(theta_goal)

        # If we press control + C, the node will stop.
		sleep(1)



def main():
	# initialize the node
	rospy.Subscriber('task1_goals', PoseArray, task1_goals_Cb)
	print("=============================")
	print("Starting controller node")
	print("=============================")
	bot = HolaBot()

	sleep(2)
	print("Starting to move")
	for goal in zip(x_goals, y_goals, theta_goals):
		bot.move2goal(goal[0], goal[1], goal[2])
	print("Finished moving")
	rospy.signal_shutdown("Finished moving")
	# Initialze Node
	# We'll leave this for you to figure out the syntax for 
	# initialising node named "controller"
	
	# Initialze Publisher and Subscriber
	# We'll leave this for you to figure out the syntax for
	# initialising publisher and subscriber of cmd_vel and odom respectively

	# Declare a Twist message
    # vel = Twist()
	# # Initialise the required variables to 0
	# # <This is explained below>
	
	# # For maintaining control loop rate.
    # rate = rospy.Rate(100)

	# Initialise variables that may be needed for the control loop
	# For ex: x_d, y_d, theta_d (in **meters** and **radians**) for defining desired goal-pose.
	# and also Kp values for the P Controller

	#
	# 
	# Control Loop goes here
    #
    # while not rospy.is_shutdown():

	# 	# Find error (in x, y and theta) in global frame
	# 	# the /odom topic is giving pose of the robot in global frame
	# 	# the desired pose is declared above and defined by you in global frame
	# 	# therefore calculate error in global frame

	# 	# (Calculate error in body frame)
	# 	# But for Controller outputs robot velocity in robot_body frame, 
	# 	# i.e. velocity are define is in x, y of the robot frame, 
	# 	# Notice: the direction of z axis says the same in global and body frame
	# 	# therefore the errors will have have to be calculated in body frame.
	# 	# 
	# 	# This is probably the crux of Task 1, figure this out and rest should be fine.

	# 	# Finally implement a P controller 
	# 	# to react to the error with velocities in x, y and theta.

	# 	# Safety Check
	# 	# make sure the velocities are within a range.
	# 	# for now since we are in a simulator and we are not dealing with actual physical limits on the system 
	# 	# we may get away with skipping this step. But it will be very necessary in the long run.

    #     vel.linear.x = vel_x
    #     vel.linear.y = vel_y
    #     vel.angular.z = vel_z

    #     pub.publish(vel)
    #     rate.sleep()



if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass
