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

x_goals = []
y_goals = []
theta_goals = []

hola_x = 0
hola_y = 0
hola_theta = 0


def task1_goals_Cb(msg):
	global x_goals, y_goals, theta_goals

	x_goals.clear()
	y_goals.clear()
	theta_goals.clear()

	for waypoint_pose in msg.poses:
		print(waypoint_pose)
		x_goals.append(waypoint_pose.position.x)
		y_goals.append(waypoint_pose.position.y)

		orientation_q = waypoint_pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		theta_goal = euler_from_quaternion (orientation_list)[2]
		theta_goals.append(theta_goal)


def odomCb(msg: Odometry):
    # get the position and orientation from the odometry message
    global hola_x, hola_y, hola_theta
    hola_x = round(msg.pose.pose.position.x,4)
    hola_y = round(msg.pose.pose.position.y,4)
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    hola_theta = euler_from_quaternion (orientation_list)[2]


#node initialization
rospy.init_node('controller', anonymous=True)
#subscribe to odom
rospy.Subscriber('/odom', Odometry, odomCb)
#subscribe to task1_goals
rospy.Subscriber('/task1_goals', PoseArray, task1_goals_Cb)

#publish to cmd_vel
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
#rate 
rate = rospy.Rate(100) # 100hz


def euclidean_distance(goal_x, goal_y):
    global hola_x, hola_y
    return math.sqrt((goal_x - hola_x)**2 + (goal_y - hola_y)**2)


def steering_angle(goal_x, goal_y):
    global hola_x, hola_y, hola_theta
    return math.atan2(goal_y - hola_y, goal_x - hola_x)


def linear_vel_x(goal_x, goal_y, constant=2):
    global hola_x, hola_y, hola_theta
    return constant * math.cos(steering_angle(goal_x, goal_y)-hola_theta)

def linear_vel_y(goal_x, goal_y, constant=2):
    global hola_x, hola_y, hola_theta
    return constant * math.sin(steering_angle(goal_x, goal_y)-hola_theta)


def angular_vel(goal_theta, constant=3):
    return constant * (goal_theta-hola_theta)

def angle_range(theta):
    theta = theta%(2*math.pi) 
    if theta > math.pi or theta < -math.pi:
        theta = theta - 2*math.pi if theta > 0 else theta + 2*math.pi
    return theta

def rotate(goal_theta,angular_speed=1.5):
    global hola_theta

    goal_theta = angle_range(goal_theta)

    print("goal_theta: {}".format(goal_theta))
    
    msg = Twist()
    angle_tolerance = 0.01

    msg.linear.x = 0
    msg.linear.y = 0
    msg.linear.z = 0
    msg.angular.x = 0
    msg.angular.y = 0

    while abs(goal_theta - hola_theta) >= angle_tolerance:
        msg.angular.z = angular_speed if goal_theta > hola_theta else -angular_speed
        pub.publish(msg)
        rate.sleep()
    msg.angular.z = 0
    pub.publish(msg)

    rate.sleep()


def move2goal(goal_x, goal_y, goal_theta):
    global hola_x, hola_y, hola_theta

    print("goal_x: {}, goal_y: {}, goal_theta: {}".format(goal_x, goal_y, goal_theta))
    
    goal_theta = angle_range(goal_theta)
    distance_tolerence = 0.01
    msg = Twist()

    msg.angular.z = 0
    while euclidean_distance(goal_x, goal_y) >= distance_tolerence:
        msg.linear.x = linear_vel_x(goal_x, goal_y, .5)
        msg.linear.y = linear_vel_y(goal_x, goal_y, .5)
        msg.angular.z = angular_vel(goal_theta)
        pub.publish(msg)
        print(goal_x, goal_y, hola_x, hola_y)
        rate.sleep()

    msg.linear.x = 0
    msg.linear.y = 0
    msg.angular.z = 0
    pub.publish(msg)

    rate.sleep()


def main():
    global x_goals, y_goals, theta_goals, hola_x, hola_y, hola_theta
    
    j = 1

    test_cases = 3

    while not rospy.is_shutdown() :
        if x_goals and y_goals and theta_goals:
            for goal_x, goal_y, goal_theta in zip(x_goals, y_goals, theta_goals):
                move2goal(goal_x, goal_y, goal_theta)
                rotate(goal_theta)
                rospy.loginfo("Reached goal: {}, {}, {}".format(goal_x, goal_y, goal_theta))
                sleep(3)
            x_goals.clear()
            y_goals.clear()
            theta_goals.clear()
            # j+=1
    rospy.signal_shutdown("Test cases completed")
    




if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
