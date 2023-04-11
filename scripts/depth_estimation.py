#!/usr/bin/env python

import math
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from darknet_ros_msgs.msg import BoundingBoxes
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np

HORIZONTAL_ANGLE = np.pi / 180 * 70.42 #62.2
IMAGE_WIDTH = 640 #1920

current_laserscan = LaserScan()
current_boundingboxes = BoundingBoxes()
current_yaw = 0
current_position_x = 0
current_position_y = 0

def add_text_to_markers(shape_markers, i):
    markers = []
    for shape_marker in shape_markers.markers:
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "my_namespace"
        marker.id = i
        marker.type = marker.TEXT_VIEW_FACING
        marker.action = marker.ADD
        marker.text = shape_marker.text
        marker.pose.position.x = shape_marker.pose.position.x
        marker.pose.position.y = shape_marker.pose.position.y
        marker.pose.position.z = 0
        marker.pose.orientation.w = 1
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1
        marker.color.a = 1
        marker.color.r = 1
        marker.color.g = 1
        marker.color.b = 1
        marker.lifetime = rospy.Duration(3)
            
        markers.append(marker)
        i += 1
    
    return markers
    

def position_to_angle(position):
    global IMAGE_WIDTH
    global HORIZONTAL_ANGL
    angle = position / IMAGE_WIDTH * HORIZONTAL_ANGLE - HORIZONTAL_ANGLE / 2
    if angle < 0:
        angle += np.pi*2
    return angle

def calculate_depth():
    global current_boundingboxes
    global current_laserscan
    global current_position_x
    global current_position_y
    boxes = current_boundingboxes
    laserscan = current_laserscan
    marker_list = MarkerArray()
    for (i, box) in enumerate(boxes.bounding_boxes):
        angle_min = position_to_angle(box.xmin)
        angle_max = position_to_angle(box.xmax)
        if angle_min > angle_max:
            tmp = angle_max
            angle_max = angle_min
            angle_min = tmp
        angle_center = (angle_min + angle_max) / 2
        ranges = []
        for (j, range) in enumerate(laserscan.ranges):
            angle = laserscan.angle_min + laserscan.angle_increment * j
            if (angle_min > math.pi and angle_max > math.pi or angle_max < math.pi and angle_min < math.pi) and angle >= angle_min and angle <= angle_max and range >= laserscan.range_min and range <= laserscan.range_max:
                ranges.append(range)
            elif (angle_max > math.pi and angle_min < math.pi and angle >= angle_max or angle <= angle_min) and range >= laserscan.range_min and range <= laserscan.range_max:
                ranges.append(range)
        rospy.loginfo(ranges)
        if (ranges != []):
            mean_range = np.mean(ranges)
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "my_namespace"
            marker.id = i
            marker.type = marker.CUBE
            marker.action = marker.ADD
            marker.text = box.Class
            marker.pose.position.y = math.sin(angle_center) * mean_range
            
            marker.pose.position.x = math.cos(angle_center) * mean_range

            marker.pose.position.z = 0
            marker.pose.orientation.w = 1
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1
            marker.color.r = 0
            marker.color.g = 0
            marker.color.b = 1
            marker.lifetime = rospy.Duration(3)
            
            marker_list.markers.append(marker)
        else:
            marker_list.markers = []
            rospy.loginfo("No ranges detected")
    # marker_list.markers.extend(add_text_to_markers(marker_list, len(marker_list.markers)))
    rospy.loginfo(marker_list)
    pub.publish(marker_list)

def boundingbox_recieved(boxes):
    global current_boundingboxes
    current_boundingboxes = boxes
    calculate_depth()

def laserscan_recieved(laserscan):
    global current_laserscan
    current_laserscan = laserscan

def odom_recieved(odom):
    global current_position_x
    global current_position_y
    global current_yaw
    orientation_q = odom.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    current_yaw = yaw
    current_position_x = odom.pose.pose.position.x
    current_position_y = odom.pose.pose.position.y


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
