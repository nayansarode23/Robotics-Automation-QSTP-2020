#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

rospy.init_node("obstacle_detect")
pub = rospy.Publisher("obstacle", Path, queue_size=5, latch=True)
list1 = [
    [0, 1.5],
    [0, 3],
    [0, 4.5],
    [1.5, 0],
    [1.5, 1.5],
    [1.5, 3],
    [1.5, 4.5],
    [3, 0],
    [3, 1.5],
    [3, 3],
    [3, 4.5],
    [4.5, 0],
    [4.5, 1.5],
    [4.5, 3],
    [4.5, 4.5],
]
path = Path()
path.header.frame_id = str(len(list1))
for i in list1:
    pose = PoseStamped()
    pose.pose.position.x = i[0]
    pose.pose.position.y = i[1]
    pose.pose.position.z = 0
    path.poses.append(pose)

pub.publish(path)
rospy.spin()
