#!/usr/bin/env python

import cmath
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import numpy as np

RASPBERRY_PI_ANGLE = np.pi * 62.2 / 180
RASPBERRY_PI_WIDTH = 2592
RASPBERRY_PI_HEIGHT = 1944

current_laserscan = LaserScan()
current_boundingboxes = BoundingBoxes()

def position_to_angle(position):
    return (RASPBERRY_PI_ANGLE / 2) - ((RASPBERRY_PI_WIDTH - position) / RASPBERRY_PI_WIDTH) * RASPBERRY_PI_ANGLE

def calculate_depth():
    marker_list = MarkerArray()
    global current_boundingboxes
    global current_laserscan
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
            rospy.loginfo(angle)
            if (angle >= angle_min and angle <= angle_max and range >= laserscan.range_min and range <= laserscan.range_max):
                ranges += range
        rospy.loginfo(ranges)
        mean_range = np.mean(ranges)
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.get_time()
        marker.ns = "my_namespace"
        marker.id = i
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.points.append(Point(cmath.sin(angle_center) * mean_range, cmath.cos(angle_center) * mean_range), 0)
        #marker.pose.position.x = cmath.sin(angle_center) * mean_range
        #marker.pose.position.y = cmath.cos(angle_center) * mean_range
        #marker.pose.position.z = 0
        #marker.pose.orientation.x = 0.0
        #marker.pose.orientation.y = 0.0
        #marker.pose.orientation.z = 0.0
        #marker.pose.orientation.w = 1.0
        #marker.scale.x = 1
        #marker.scale.y = 0.1
        #marker.scale.z = 0.1
        #marker.color.a = 1.0
        #marker.color.r = 1.0
        #marker.color.g = 0.0
        #marker.color.b = 0.0
        marker_list.markers.append(marker)   
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
    #calculate_depth()


if __name__ == '__main__':
    try:
        pub = rospy.Publisher('depth_estimation', MarkerArray, queue_size=20)
        rospy.init_node('depth_estimation', anonymous=True)
        rospy.Subscriber("scan", LaserScan, laserscan_recieved)
        rospy.Subscriber("darknet_ros/bounding_boxes", BoundingBoxes, boundingbox_recieved)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass