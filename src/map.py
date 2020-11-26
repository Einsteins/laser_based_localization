# This script is used to establish map.txt , it subscribe 
# map topic and generate the txt data
#!/usr/bin/env python

import cv2
import numpy as np
import trimesh
import matplotlib.pyplot as plt

from mpl_toolkits.mplot3d import Axes3D
from matplotlib.tri import Triangulation
from geometry_msgs.msg import PoseArray, Pose
import rospy
from nav_msgs.msg import OccupancyGrid

import math





#print length
# plt.scatter(width,length)
# plt.scatter(0,0,c='r')
#
#
# plt.savefig('map.png')
# plt.show()
# cv2.imshow('gray',img_2[1])
def map_preprocessing(map_msg):
    #rospy.init_node('preprocessing_map')
    map_dims = (map_msg.info.height, map_msg.info.width)
    map_array = np.array(map_msg.data).reshape(map_dims)

    map_array[map_array < 0] = 0
    map_array =map_array.astype(np.uint8)
    _,thresh_map = cv2.threshold(map_array, 1, 100, cv2.THRESH_BINARY)
    image, contours, hierarchy = cv2.findContours(thresh_map, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
    hierarchy =hierarchy[0]
    x=[]
    y=[]
    a=[]
    b=[]
    new_x=[]
    new_y=[]
    for i in range(len(contours)):
        for j in range(len(contours[i])):
            x.append(contours[i][j][0][0]*0.05-15)
            y.append(contours[i][j][0][1]*0.05-13.5)


    #print len(x)
    #plt.scatter(x,y)

    f = open('gazebo_map.txt','w')
    f.writelines(str(x)+'\n')

    f.write(str(y)+'\n')
    f.close()
    # plt.savefig('new_map_original_3382.png')



    # group,number =cluster(x,y)
    # print number
    # detected_center = []
    # detected_center_x = []
    # detected_center_y = []
    # for i in range(number):
    #     center_x = sum(group[i].x) / len(group[i].x)
    #     center_y = sum(group[i].y) / len(group[i].y)
    #     detected_center_x.append(center_x)
    #     detected_center_y.append(center_y)
    # plt.scatter(detected_center_x,detected_center_y)
    # plt.show()

    for i in range(len(x)):
        x1 = round(x[i], 1)
        y1 = round(y[i], 1)
        a.append(x1)
        b.append(y1)


    a = np.array(a)
    b = np.array(b)
    test_a = a
    test_b = b

    for i in range(len(test_a) - 1):
        if test_a[i] == test_a[i + 1] and test_b[i] == test_b[i + 1]:
            a[i] = 999
            b[i] = 999
        # elif (test_a[i] -8)**2 +(test_b[i]-7)**2 > 49:
        #     a[i] = 999
        #     b[i] =999
        elif test_a[i] > 22 or test_a[i]<1 or test_b[i] >13.5 or test_b[i]<0.3:
            a[i] = 999
            b[i] = 999
        # elif test_a[i] >2 and test_a[i] <5 and test_b[i] <9 and test_b[i] >7:
        #     a[i] =999
        #     b[i] =999
        else:
            continue
    new_x = []
    new_y = []

    for i in range(len(a)):
        if a[i] < 999:
            new_x.append(a[i])
            new_y.append(b[i])
        else:
            continue

    filt_x=[]
    filt_y=[]
    for i in range(0,len(new_x),3):
        filt_x.append(new_x[i])
        filt_y.append(new_y[i])





    #
    print len(filt_x)
    #f = open('filtered_map.txt','w')
    #f.writelines(str(filt_x)+'\n')

    #f.write(str(filt_y)+'\n')
    #f.close()


    #plt.scatter(filt_x,filt_y)


    #plt.savefig("filtered_new_map.png",dpi =600)
    plt.show()












if __name__ == '__main__':
    rospy.init_node("map2")
    map_topic = rospy.get_param("~map_topic", "map")
    occupied_thresh = rospy.get_param("~occupied_thresh", 1)
    box_height = rospy.get_param("~box_height", 2.0)
    rospy.Subscriber(map_topic,OccupancyGrid,map_preprocessing)
    rospy.spin()
    # rospy.init_node('preprocessing_map')
    # img1 = cv2.imread('map20Feb20201.pgm', 1)
    # img_gray = cv2.cvtColor(img1, cv2.COLOR_RGB2GRAY)
    # img_2 = cv2.threshold(img_gray, 1, 100, cv2.THRESH_BINARY)
    #
    # img_3 = img_2[1]
    # length = []
    # width = []
    # for i in range(len(img_3)):
    #     for j in range(len(img_3)):
    #         if img_3[i][j] != 100:
    #             length.append(-(i * 0.05000 - 34.400))
    #             width.append(j * 0.05000 - 68.000)
    # x=width
    # y=length
    #
    # Pub = rospy.Publisher('map_point', PoseArray, queue_size=1)
    #
    #
    # Point = PoseArray()
    # msg_sub =[]
    # #print len(length)
    #
    # for i in range(len(length)):
    #     msg = Pose()
    #     msg.position.x = width[i]
    #     msg.position.y = length[i]
    #     msg.position.z = 0
    #     msg.orientation.x = 0
    #     msg.orientation.y = 0
    #     msg.orientation.z = 0
    #     msg.orientation.w = 0
    #     msg_sub.append(msg)
    #
    # Point.poses =msg_sub
    # Point.header.stamp = rospy.Time.now()
    # Point.header.frame_id = 'map'
    # while not rospy.is_shutdown():
    #     Pub.publish(Point)
    #     rospy.loginfo('publish')
        #rospy.sleep(1)








