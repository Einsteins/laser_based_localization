#!/usr/bin/env python

# this script subscribe f_scan and b_scan of master robot and use
# tf information fusion, then publish to topic /master_scan_fusion with Hz 10

import rospy
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from geometry_msgs.msg import PointStamped
import tf
import message_filters
import sensor_msgs.point_cloud2 as pc2
from laser_geometry import LaserProjection
from laser_based_localization.msg import Array
from laser_based_localization.msg import ArrayOfArrays




def transform(child, parent, point):  # transform point from child to parent coordinate

    listener.waitForTransform(child, parent, rospy.Time(0), rospy.Duration(4.0))
    laser_point = PointStamped()
    laser_point.header.frame_id = child
    laser_point.header.stamp = rospy.Time(0)
    laser_point.point.x = point[0]
    laser_point.point.y = point[1]
    laser_point.point.z = 0
    p = listener.transformPoint(parent, laser_point)

    return p


def laser_fusion(f_data, b_data):
    # read data from front laser

    laser_max_range = 29   # maximal laser range

    laserProj = LaserProjection()
    child = rospy.get_param("~front_link")
    parent= rospy.get_param("~map")

    cloud_out = laserProj.projectLaser(f_data,laser_max_range)
    points = pc2.read_points(cloud_out)
    x_f = []
    y_f = []

    for point in points:
        p = transform(child, parent, point)
        x_f.append(p.point.x)
        y_f.append(p.point.y)

    # read data from back laser

    child = rospy.get_param("~back_link")

    cloud_out = laserProj.projectLaser(b_data)
    points = pc2.read_points(cloud_out)
    x_b = []
    y_b = []
    for point in points:
        p = transform(child, parent, point)
        x_b.append(p.point.x)
        y_b.append(p.point.y)

    x_f.extend(x_b)
    y_f.extend(y_b)
    msg_x = Array()
    msg_x.array = x_f
    msg_y = Array()
    msg_y.array = y_f
    msg = ArrayOfArrays()
    msg.array_of_arrays = [msg_x, msg_y]
    rate = rospy.Rate(10)      #defeind the publishe rate 10 hz
    pub.publish(msg)
    rate.sleep()



if __name__ == '__main__':
    rospy.init_node("scan_fusion")
    listener = tf.TransformListener()
    sub_b = message_filters.Subscriber(rospy.get_param("~f_scan"), LaserScan)
    sub_f = message_filters.Subscriber(rospy.get_param("~b_scan"), LaserScan)
    ts = message_filters.ApproximateTimeSynchronizer([sub_b, sub_f], queue_size=1, slop=0.1, allow_headerless=False) # slop is the acceptable time difference between two topics
    ts.registerCallback(laser_fusion)
    pub = rospy.Publisher("/robots_scan_fusion", ArrayOfArrays, queue_size=1)
    rospy.spin()