#!/usr/bin/env python2.7
"""
ROS node performing wall following for the F1Tenth simulator
Based on the code from the official F1Tenth lab n°3 at https://github.com/f1tenth/f1tenth_labs/tree/main/lab3
"""
from __future__ import print_function
import sys
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

#PID CONTROL PARAMETERS
kp = -70
kd = -30
ki = 0
servoOffset = 0.0
prevError = 0.0 
error = 0.0
integral = 0.0

#WALL FOLLOW PARAMS
ANGLE_RANGE = 270 # UST-10LX has a 270 degrees scan angle
DESIRED_DISTANCE_RIGHT = 0.9 # In meters
DESIRED_DISTANCE_LEFT = 1 # Needs to be adjusted depending on the track's width
VELOCITY = 5.00 # In m/s
CAR_LENGTH = 0.50

class WallFollow:
    def __init__(self):
        # Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/drive'
        self.f = open('data.csv','w')
        self.odometry = None
        self.imu = None
        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback, queue_size = 1)
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size = 1)
        self.odom_subscriber = rospy.Subscriber("/odom", Odometry, self.odometry_callback)
        self.imu_subscriber = rospy.Subscriber("/imu", Imu, self.imu_callback)

    def getRange(self, data, angle):
        # data: single message from topic /scan
        # angle: between -45 to 225 degrees, where 0 degrees is directly to the right
        # Outputs length in meters to object with angle in LiDAR scan field of view
        # Make sure to take care of nans etc.
        ranges = np.array(data.ranges)
        angle_incr = data.angle_increment
        desired_idx = int((np.radians(angle)-data.angle_min)/angle_incr)
        if np.isfinite(ranges[desired_idx]):
            return ranges[desired_idx]
        else:
            return None



    def pidControl(self, error):
        global integral
        global prevError
        global kp
        global ki
        global kd
        deriv_error = error-prevError
        prevError = error
        integral += error
        angle = np.radians(kp*error+kd*deriv_error+ki*integral)
        velocity = self.calcSpeed(angle)
        #rospy.loginfo("Error {}, Angle {}".format(error, np.degrees(angle)))
        self.f.write('\n' + str(rospy.Time.now()) + ',' + str(np.degrees(angle)) + ',' + str(self.get_car_linear_velocity()) + ',' + str(self.get_car_linear_acceleration()))
        driveMsg = AckermannDriveStamped()
        driveMsg.header.stamp = rospy.Time.now()
        driveMsg.header.frame_id = "laser"
        driveMsg.drive.steering_angle = angle
        driveMsg.drive.speed = velocity
        self.drive_pub.publish(driveMsg)

    def calcSpeed(self, angle):
        angle = np.abs(np.degrees(angle))
        if angle >= 0 and angle < 10:
            speed = VELOCITY
        elif angle >= 10 and angle < 20:
            speed = VELOCITY/2
        else:
            speed = VELOCITY/4
        return speed

    def followLeft(self, data, leftDist):
        # Follow the left wall according to the algorithm 

        zeroAngle = 90
        b = self.getRange(data, zeroAngle)
        
        theta = 35 # Arbitrary value around 40°
        a = self.getRange(data, zeroAngle - theta)
        theta = np.radians(theta)
        if b is not None and a is not None:
            alpha = np.arctan2(a*np.cos(theta)-b, a*np.sin(theta))
            Dleft = b*np.cos(alpha)
            distLeftLookahead = Dleft+CAR_LENGTH*np.sin(alpha)
            error = DESIRED_DISTANCE_LEFT - distLeftLookahead
            return error
        else:
            return None

    def lidar_callback(self, data):
        error = self.followLeft(data, DESIRED_DISTANCE_LEFT)
        if error is not None:
            self.pidControl(error)  # Send the error to pidControl

    def odometry_callback(self, odometry):
        self.odometry = odometry

    def imu_callback(self, imu):
        self.imu = imu

    def get_car_linear_velocity(self):
        if self.odometry is None or (self.odometry.twist.twist.linear.x == 0 and self.odometry.twist.twist.linear.x == 0):
            return 0
        return np.sqrt(self.odometry.twist.twist.linear.x ** 2 + self.odometry.twist.twist.linear.y ** 2)

    def get_car_angular_acceleration(self):
        return self.odometry.twist.twist.angular

    def get_car_linear_acceleration(self):
        return np.sqrt(self.imu.linear_acceleration.x ** 2 + self.imu.linear_acceleration.y ** 2)

def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)