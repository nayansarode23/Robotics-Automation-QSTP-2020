#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


def callback(msg):
    pass


def callback_path(msg):
    global o
    global x_p
    global y_p
    global flag
    global l
    l = len(msg.poses)
    if l == 0:
        pass
    else:
        x = msg.poses[flag].pose.position.x
        y = msg.poses[flag].pose.position.y
    if o == 0:
        o = 1
        sub_odom = rospy.Subscriber("odom", Odometry, callback_odom)


def callback_odom(msg):
    global l
    global x
    global y
    global o_x_sum
    global o_x_prev
    global o_y_sum
    global o_y_prev
    global count1
    global count2
    global flag
    twist = Twist()
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    if count1 == 0:
        x = x_p - x
        o_x_sum += o_x
        det_x = o_x - o_x_prev
        v_x = 0.5 * o_x + 0.0008 * o_x_sum + 0.17 * dedt_x
        v_x *= 1.2
        if v_x > 0.7:
            v_x = 0.7
        elif v_x < -0.7:
            v_x = -0.7
        elif v_x < 0.05 and v_x > 0:
            v_x *= 3
        elif v_x > -0.05 and v_x < 0:
            v_x *= 3
        twist.linear.x = v_x
        pub.publish(twist)
        o_x_prev = o_x
        if o_x <= 0.05 and o_x >= -0.05:
            twist.linear.x = 0
            pub.publish(twist)
            count1 = 1
    if count2 == 0:
        o_y = y_p - y
        o_y_sum += o_y
        dedt_y = o_y - o_y_prev
        u_y = 0.5 * o_y + 0.0008 * o_y_sum + 0.17 * det_y
        u_y *= 1.2
        if u_y > 0.7:
            u_y = 0.7
        elif u_y < -0.7:
            u_y = -0.7
        elif u_y < 0.05 and u_y >= 0:
            u_y *= 3
        elif u_y > -0.05 and u_y <= 0:
            u_y *= 3
        twist.linear.y = u_y
        pub.publish(twist)
        o_y_prev = e_y
        if o_y <= 0.05 and o_y >= -0.05:
            twist.linear.y = 0
            pub.publish(twist)
            count2 = 1
    if count1 == 1 and count2 == 1:
        if flag + 1 < l:
            count1 = 0
            count2 = 0
            flag += 1
            rospy.sleep(0.1)


global count1
count1 = 0
global count2
count2 = 0
global o_x_sum
o_x_sum = 0
global o_x_prev
o_x_prev = 0
global o_y_sum
o_y_sum = 0
global o_y_prev
o_y_prev = 0
global flag
flag = 0
global o
o = 0
rospy.init_node("controller")
sub_path = rospy.Subscriber("path", Path, callback_path)
pub = rospy.Publisher("cmd_vel", Twist, queue_size=2)
rospy.spin()
