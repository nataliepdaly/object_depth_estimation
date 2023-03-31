#!/usr/bin/env python

import math
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from darknet_ros_msgs.msg import BoundingBoxes
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import numpy as np

HORIZONTAL_ANGLE = np.pi * 70.42 / 180
IMAGE_WIDTH = 720 #1920
IMAGE_HEIGHT = 1080

current_laserscan = LaserScan()
current_boundingboxes = BoundingBoxes()
current_position = Odometry()

def position_to_angle(position):
    return position / IMAGE_WIDTH * HORIZONTAL_ANGLE - HORIZONTAL_ANGLE / 2

def calculate_depth():
    global current_boundingboxes
    global current_laserscan
    global current_position
    boxes = current_boundingboxes
    laserscan = current_laserscan
    rospy.loginfo("calculating depth...")
    #rospy.loginfo(boxes.bounding_boxes)
    rospy.loginfo(laserscan.angle_min)
    rospy.loginfo(laserscan.angle_max)
    for (i, box) in enumerate(boxes.bounding_boxes):
        rospy.loginfo("bpxes found")
        #print(box.xmin)
        #print(box.xmax)
        angle_min = position_to_angle(box.xmin)
        angle_max = position_to_angle(box.xmax)
        rospy.loginfo(angle_min)
        rospy.loginfo(angle_max)
        angle_center = (angle_min + angle_max) / 2
        ranges = []
        for (i, range) in enumerate(laserscan.ranges):
            angle = laserscan.angle_min + laserscan.angle_increment * i
            if angle >= np.pi:
                angle -= np.pi*2
            #if range != float("inf"):
                #rospy.loginfo(angle)
                #rospy.loginfo(range)
            if (angle >= angle_min and angle <= angle_max and range >= laserscan.range_min and range <= laserscan.range_max):
                ranges.append(range)
        rospy.loginfo(ranges)
        marker_list = MarkerArray()
        if (ranges != []):
            mean_range = np.mean(ranges)
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "my_namespace"
            marker.id = i
            marker.type = marker.CUBE
            marker.action = marker.ADD
            marker.pose.position.x = math.cos(angle_center) * mean_range + current_position.pose.pose.position.x
            marker.pose.position.y = math.sin(angle_center) * mean_range + current_position.pose.pose.position.y
            marker.pose.position.z = 0
            marker.pose.orientation.w = 1
            marker.scale.x = 1
            marker.scale.y = 1
            marker.scale.z = 1
            marker.color.a = 1
            marker.color.r = 1
            marker.color.g = 0
            marker.color.b = 0
            marker.lifetime = rospy.Duration()
            
            marker_list.markers.append(marker)
        else:
            rospy.loginfo("No ranges detected")
    print(current_position)
    rospy.loginfo(marker_list)
    pub.publish(marker_list)

def boundingbox_recieved(boxes):
    global current_boundingboxes
    rospy.loginfo("bounding box recieved")
    rospy.loginfo(boxes)
    current_boundingboxes = boxes
    calculate_depth()

def laserscan_recieved(laserscan):
    global current_laserscan
    #rospy.loginfo(current_laserscan)
    current_laserscan = laserscan

def odom_recieved(odom):
    global current_position
    current_position = odom


if __name__ == '__main__':
    try:
        pub = rospy.Publisher('depth_estimation', MarkerArray, queue_size=20)
        rospy.init_node('depth_estimation', anonymous=True)
        rospy.Subscriber("scan", LaserScan, laserscan_recieved)
        rospy.Subscriber("darknet_ros/bounding_boxes", BoundingBoxes, boundingbox_recieved)
        rospy.Subscriber("odom", Odometry, odom_recieved)
        rospy.loginfo("Waiting for messages...")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass