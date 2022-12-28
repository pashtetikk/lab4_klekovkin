import rospy
import time
from robot import robot
from robot import ray

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt

import numpy as np
import math

def euler_from_quaternion(x, y, z, w):

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return yaw_z, pitch_y, roll_x  # in radians

def odom_callback(msg):
    robot_pose = msg
    robot_obj.set_center(robot_pose.pose.pose.position.x, robot_pose.pose.pose.position.y)
    
    robot_yaw, robot_pitch, robot_roll = euler_from_quaternion(robot_pose.pose.pose.orientation.x,
                                                                robot_pose.pose.pose.orientation.y,
                                                                robot_pose.pose.pose.orientation.z,
                                                                robot_pose.pose.pose.orientation.w)
    #print(robot_yaw)
    robot_obj.alpha = robot_yaw


def lidar_calback(msg):
    lidarRays = msg
    robot_obj.set_rays(lidarRays.ranges, lidarRays.angle_min, lidarRays.angle_increment)



if __name__ == '__main__':
    rospy.init_node('lab4_node')
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.Subscriber('/m2wr/laser/scan', LaserScan, lidar_calback)
    robot_obj = robot(0.5)
    robot_obj.set_tgt_point(-5, -5)
    cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10 )
    deb_pub = rospy.Publisher('deb_pub', String, queue_size=10)
    point_pub = rospy.Publisher('point_pub', Point, queue_size=10)
    
    while 1:
        #deb_pub.publish()
        robot_obj.handler()
        cmd = Twist()
        #print(robot_obj.getSpeed())
        cmd.linear.x, cmd.angular.z = robot_obj.getSpeed()
        cmd_pub.publish(cmd)
        #rospy.sleep(0.2)

