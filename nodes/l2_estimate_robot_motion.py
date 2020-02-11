#!/usr/bin/env python
from __future__ import division, print_function
import time

import numpy as np
import rospy
import tf_conversions
import tf2_ros

# msgs
from turtlebot3_msgs.msg import SensorState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, TransformStamped, Transform, Quaternion
from std_msgs.msg import Empty

from utils import convert_pose_to_tf, euler_from_ros_quat, ros_quat_from_euler


ENC_TICKS = 4096
RAD_PER_TICK = 0.001533981
WHEEL_RADIUS = .066 / 2
BASELINE = .287 / 2


class WheelOdom:
    def __init__(self):
        # publishers, subscribers, tf broadcaster
        self.sensor_state_sub = rospy.Subscriber('/sensor_state', SensorState, self.sensor_state_cb, queue_size=1)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_cb, queue_size=1)
        self.wheel_odom_pub = rospy.Publisher('/wheel_odom', Odometry, queue_size=1)
        self.tf_br = tf2_ros.TransformBroadcaster()

        # attributes
        self.odom = Odometry()
        self.odom.pose.pose.position.x = 1e10
        self.wheel_odom = Odometry()
        self.wheel_odom.header.frame_id = 'odom'
        self.wheel_odom.child_frame_id = 'wo_base_link'
        self.wheel_odom_tf = TransformStamped()
        self.wheel_odom_tf.header.frame_id = 'odom'
        self.wheel_odom_tf.child_frame_id = 'wo_base_link'
        self.pose = Pose()
        self.pose.orientation.w = 1.0
        self.twist = Twist()
        self.last_enc_l = None
        self.last_enc_r = None
        self.last_time = None

        # reset current odometry to allow comparison with this node
        reset_pub = rospy.Publisher('/reset', Empty, queue_size=1, latch=True)
        reset_pub.publish(Empty())
        while (self.odom.pose.pose.position.x >= 1e-3 or self.odom.pose.pose.position.y >= 1e-3 or
               self.odom.pose.pose.orientation.z >= 1e-2):
            time.sleep(0.2)  # allow reset_pub to be ready to publish
        print('Robot odometry reset.')

        rospy.spin()

    def sensor_state_cb(self, sensor_state_msg):
        # Callback for whenever a new encoder message is published
        # set initial encoder pose
        if self.last_enc_l is None:
            self.last_enc_l = sensor_state_msg.left_encoder
            self.last_enc_r = sensor_state_msg.right_encoder
            self.last_time = sensor_state_msg.header.stamp
        else:
            # update calculated pose and twist with new data
            le = sensor_state_msg.left_encoder
            re = sensor_state_msg.right_encoder

            # using equations from lecture, integrate the motion forward

            dl = (le - self.last_enc_l) / ENC_TICKS * 2 * np.pi * WHEEL_RADIUS
            dr = (re - self.last_enc_r) / ENC_TICKS * 2 * np.pi * WHEEL_RADIUS

            d_dist = 0.5 * (dl + dr)
            dz = 0.5 * (dr - dl) / BASELINE

            self.pose.position.x += d_dist * np.cos(self.pose.orientation.z)
            self.pose.position.y += d_dist * np.sin(self.pose.orientation.z)

            e_angle = euler_from_ros_quat(self.pose.orientation)
            e_angle = np.array(e_angle) + np.array((0, 0, dz))
            self.pose.orientation = ros_quat_from_euler(e_angle)

            # update velocity

            dt = sensor_state_msg.header.stamp.to_sec() - self.last_time.to_sec()

            self.twist.linear.x = d_dist/dt
            self.twist.angular.z = dz/dt

            rospy.loginfo("le %s re %s dl %s dr %s dt %s dz %s", le, re, dl, dr, dt, dz)

            self.last_enc_l = le
            self.last_enc_r = re
            self.last_time = sensor_state_msg.header.stamp

            # publish the updates as a topic and in the tf tree
            current_time = rospy.Time.now()
            self.wheel_odom_tf.header.stamp = current_time
            self.wheel_odom_tf.transform = convert_pose_to_tf(self.pose)
            self.tf_br.sendTransform(self.wheel_odom_tf)

            self.wheel_odom.header.stamp = current_time
            self.wheel_odom.pose.pose = self.pose
            self.wheel_odom.twist.twist = self.twist
            self.wheel_odom_pub.publish(self.wheel_odom)

            # for testing against actual odom -- uncomment when you're ready to test!
            # print("Wheel Odom: x: %2.3f, y: %2.3f, t: %2.3f" % (
            #     self.pose.position.x, self.pose.position.y, mu[2].item()
            # ))
            # print("Turtlebot3 Odom: x: %2.3f, y: %2.3f, t: %2.3f" % (
            #     self.odom.pose.pose.position.x, self.odom.pose.pose.position.y,
            #     self.euler_from_ros_quat(self.odom.pose.pose.orientation)[2]
            # ))

    def odom_cb(self, odom_msg):
        # get odom from turtlebot3 packages
        self.odom = odom_msg


if __name__ == '__main__':
    try:
        rospy.init_node('wheel_odometry')
        wheel_odom = WheelOdom()
    except rospy.ROSInterruptException:
        pass