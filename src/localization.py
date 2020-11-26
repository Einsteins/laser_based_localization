#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseArray, Pose
from laser_geometry import LaserProjection
import message_filters
import tf
import math
from matplotlib import pyplot as plt
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import Rectangle_finder
from laser_based_localization.msg import ArrayOfArrays
import numpy as np
from clustering import cluster
from scipy.spatial.transform import Rotation as R

# This script subscribe the Topic 'robots_scan_fusion' and publish the topic 'calculated_robot_center', which
# contain position and orientation information of detected robots at publish rate 10hz.


class Points:
    def __init__(self, name, x, y):
        self.name = name
        self.x = x
        self.y = y


class position:
    def __init__(self, x, y, z, rotation_x, rotation_y, rotation_z, rotation_w):
        self.x = x
        self.y = y
        self.z = z
        self.rotation_x = rotation_x
        self.rotation_y = rotation_y
        self.rotation_z = rotation_z
        self.rotation_w = rotation_w


class Tracking():
    def __init__(self):
        # define the beginning parameter, they all constant
        self.loop_rate = rospy.Rate(10)
        self.rate =10

        # define the variable that will be updated during the program running
        self.amcl_orientation=[]
        self.master_odom=[]                           # master odometry position
        self.old_position_orientation_speed = []      # calculated last timestamp position , orientation and speed
        self.calculated_position = []                 # calculated this timestamp position
        self.calculated_orientation =[]               # calculated this timestamp orientation
        self.calculated_position_rotation_speed = []  # calculated this timestamp position , orientation and speed
        self.true_position = []                       # validated this timestamp position
        self.linear_speed = []                        # speed from odometry topic

        self.odom_position = []                       #odometry position of robots
        self.odom_last_timestamp = 0                  # initial a odometry timestamp
        self.dodmetry_time_difference= 1/self.rate    # initial odometry time difference

        self.laserProj = LaserProjection()
        self.pub = rospy.Publisher('calculated_robot_center', PoseArray, queue_size=1, tcp_nodelay=False)
        self.amcl_orientation =[0,0,0,1]             # in case it has't been published


        rospy.Subscriber('/robots_scan_fusion', ArrayOfArrays, self.laser_fusion_callback)
        rospy.Subscriber(rospy.get_param('~master_odometry'),Odometry,self.odometry_callback_master,queue_size=1)

        self.cases(number_of_robots)


    # define different situations for different number of robots

    def cases(self, robots_number):
        switcher = {
            1: self.one_robot,
            2: self.two_robots,
            3: self.three_robots,
        }
        switch_case = switcher.get(robots_number)
        return switch_case()

    def odometry_callback_master(self,odom):
        master_initial_x= rospy.get_param("~master_initial_x")
        master_initial_y = rospy.get_param("~master_initial_y")

        self.master_odom = [odom.pose.pose.position.x + master_initial_x,
                             odom.pose.pose.position.y + master_initial_y]


    def laser_fusion_callback(self, data):

        # use local variable to make sure global variable(self.) can be update completely
        calculated_position_sub = []
        calculated_orientation_sub=[]
        laser_data = data.array_of_arrays

        # get the position of  x and y from scan fusion
        x = laser_data[0].array
        y = laser_data[1].array

        group, number_of_objects = cluster(x, y)


        detected_center = []
        # calculate the center point of each group
        for i in range(number_of_objects):
            center_x = sum(group[i].x) / len(group[i].x)
            center_y = sum(group[i].y) / len(group[i].y)
            detected_center.append([center_x, center_y])

        obscale_index = []

        # if the number of these group less than 20 ,then it can't be robot, delete some noise
        for i in range(number_of_objects):
            if len(group[i].x) < 20:
                obscale_index.append(i)
            else:
                for j in range(len(map[0])):
                    # calculate the distance of center point of each group to the points on map,
                    # if distance <1 m, consider it as the obstacle
                    distance_element = np.sqrt(
                        (detected_center[i][0] - map[0][j]) ** 2 + (detected_center[i][1] - map[1][j]) ** 2)

                    if distance_element < 1:
                        obscale_index.append(i)
                        break
                    else:
                        continue

        # Until here , all obstacles should be eliminated
        number_of_objects = np.array(range(number_of_objects))
        obscale_index = np.array(obscale_index)
        filtered_number_of_objects = np.delete(number_of_objects, obscale_index)
        filtered_x = []
        filtered_y = []

        for i in range(len(filtered_number_of_objects)):

            filtered_x.extend(group[filtered_number_of_objects[i]].x)
            filtered_y.extend(group[filtered_number_of_objects[i]].y)


        if len(filtered_number_of_objects) != number_of_robots:
            rospy.loginfo("error when cluster the robots")
        else:
            for i in range(len(filtered_number_of_objects)):
                realibility_threshold=0.3
                a, c, realibility = Rectangle_finder.rectangular(group[filtered_number_of_objects[i]].x,
                                                                 group[filtered_number_of_objects[i]].y)
                if realibility>realibility_threshold:
                    calculated_position_element = [c[0], c[1]]
                    r = R.from_euler('xyz', [0, 0, a], degrees=False)

                    calculated_orientation_element = r.as_quat()
                    calculated_position_sub.append(calculated_position_element)
                    calculated_orientation_sub.append(calculated_orientation_element)
                else:
                    master_x=self.master_odom[0]
                    master_y=self.master_odom[1]
                    # calculate the distane of possible robot position to master robot
                    d0 = math.sqrt((master_x-c[0][0])**2+(master_y-c[0][1])**2)
                    d1 = math.sqrt((master_x - c[1][0])**2 + (master_y - c[1][1])**2)
                    if d0<d1:   # true position is c[1]
                        calculated_position_element = [c[1][0], c[1][1]]
                        r = R.from_euler('xyz', [0, 0, a], degrees=False)

                        calculated_orientation_element = r.as_quat()
                        calculated_position_sub.append(calculated_position_element)
                        calculated_orientation_sub.append(calculated_orientation_element)
                    else:     # true position is c[0]
                        calculated_position_element = [c[0][0], c[0][1]]
                        r = R.from_euler('xyz', [0, 0, a], degrees=False)
                        calculated_orientation_element = r.as_quat()
                        calculated_position_sub.append(calculated_position_element)
                        calculated_orientation_sub.append(calculated_orientation_element)

            # Now all calculated position and orientation are saved in global parameter
            self.calculated_position = calculated_position_sub
            self.calculated_orientation=calculated_orientation_sub
    def one_robot(self):

        # get initial position from tf odom map
        robot1_initial_x = rospy.get_param("~robot1_initial_x")
        robot1_initial_y = rospy.get_param("~robot1_initial_y")
        self.robot1_initial_position = Points('robot1',robot1_initial_x , robot1_initial_y)
        rospy.Subscriber(rospy.get_param("~robot1_odometry"),Odometry,self.odometry_callback_1robot,queue_size=1)



    def two_robots(self):
        # get initial position from tf odom map

        robot1_initial_x = rospy.get_param("~robot1_initial_x")
        robot1_initial_y = rospy.get_param("~robot1_initial_y")
        robot2_initial_x = rospy.get_param("~robot2_initial_x")
        robot2_initial_y = rospy.get_param("~robot2_initial_y")
        self.robot1_initial_position = Points('robot1', robot1_initial_x, robot1_initial_y)
        self.robot1_initial_position = Points('robot2', robot2_initial_x, robot2_initial_y)
        rospy.Subscriber(rospy.get_param("~robot1_odometry"), Odometry, self.odometry_callback_1robot, queue_size=1)
        rospy.Subscriber(rospy.get_param("~robot2_odometry"), Odometry, self.odometry_callback_2robot, queue_size=1)

    def three_robots(self):
        robot1_initial_x = rospy.get_param("~robot1_initial_x")
        robot1_initial_y = rospy.get_param("~robot1_initial_y")
        robot2_initial_x = rospy.get_param("~robot2_initial_x")
        robot2_initial_y = rospy.get_param("~robot2_initial_y")
        robot3_initial_x = rospy.get_param("~robot3_initial_x")
        robot3_initial_y = rospy.get_param("~robot3_initial_y")
        self.robot1_initial_position = Points('robot1', robot1_initial_x, robot1_initial_y)
        self.robot2_initial_position = Points('robot2', robot2_initial_x, robot2_initial_y)
        self.robot3_initial_position = Points('robot2', robot3_initial_x, robot3_initial_y)
        rospy.Subscriber(rospy.get_param("~robot1_odometry"), Odometry, self.odometry_callback_1robot, queue_size=1)
        rospy.Subscriber(rospy.get_param("~robot2_odometry"), Odometry, self.odometry_callback_2robot, queue_size=1)
        rospy.Subscriber(rospy.get_param("~robot3_odometry"), Odometry, self.odometry_callback_3robot, queue_size=1)


    def odometry_callback_1robot(self,odom):
        listener.waitForTransform(rospy.get_param("~map_link"), rospy.get_param("~robot1_odometry_link"), rospy.Time(0), rospy.Duration(0.1))
        t, r = listener.lookupTransform(rospy.get_param("~map_link"), rospy.get_param("~robot1_odometry_link"), rospy.Time(0))

        # for rosbag , at the beginning , tf data not received , for program running ,use try strcture,
        # if get error, just set tf as identity matrix

        try:
            t, r = listener.lookupTransform(rospy.get_param("~map_link"), rospy.get_param("~robot1_odometry_link"), rospy.Time(0))
            b = R.from_quat(r)
            angle = b.as_euler('zyx', degrees=False)


            mat1 = [[math.cos(angle[0]), -math.sin(angle[0]), 0.0, t[0]],
                    [math.sin(angle[0]), math.cos(angle[0]), 0., t[1]],
                    [0.0, 0.0, 1.0, t[2]],
                    [0.0, 0.0, 0.0, 1.0]]
        except(tf.LookupException, tf.ConnectivityException, Exception):
            mat1 = [[1, 0, 0.0, 0],
                    [0, 1, 0., 0],
                    [0.0, 0.0, 1.0, 0],
                    [0.0, 0.0, 0.0, 1.0]]
        odom_lasttimestamp= self.odom_last_timestamp
        odom_timestamp = odom.header.stamp.secs
        self.dodmetry_time_difference = odom_lasttimestamp - odom_lasttimestamp
        odometry_position =[odom.pose.pose.position.x + self.robot1_initial_position.x,
                            odom.pose.pose.position.y + self.robot1_initial_position.y]
        odom_position =[odometry_position]
        self.odom_position =odom_position

        odom_speed_robot1 = [odom.twist.twist.linear.x  ,odom.twist.twist.linear.y ,odom.twist.twist.angular.z,0]
        odom_speed_robot1_tf = np.dot(mat1,odom_speed_robot1)
        if len(odom_speed_robot1_tf) !=0:
            odom_speed = [[odom_speed_robot1_tf[0],odom_speed_robot1_tf[1],odom_speed_robot1_tf[2]]]
        else:
            odom_speed =[[0,0,0,0]]
        self.linear_speed=odom_speed
        self.odom_last_timestamp = odom_timestamp    # update the odometry time stamp
    def odometry_callback_2robots(self, odom_2):

        listener.waitForTransform(rospy.get_param("~map_link"), rospy.get_param("~robot2_odometry_link"), rospy.Time(0),
                                  rospy.Duration(0.1))
        t, r = listener.lookupTransform(rospy.get_param("~map_link"), rospy.get_param("~robot2_odometry_link"),
                                        rospy.Time(0))

        # for rosbag , at the beginning , tf data not received , for program running ,use try strcture,
        # if get error, just set tf as identity matrix

        try:
            t, r = listener.lookupTransform(rospy.get_param("~map_link"), rospy.get_param("~robot2_odometry_link"),
                                            rospy.Time(0))
            b = R.from_quat(r)
            angle = b.as_euler('zyx', degrees=False)

            mat1 = [[math.cos(angle[0]), -math.sin(angle[0]), 0.0, t[0]],
                    [math.sin(angle[0]), math.cos(angle[0]), 0., t[1]],
                    [0.0, 0.0, 1.0, t[2]],
                    [0.0, 0.0, 0.0, 1.0]]
        except(tf.LookupException, tf.ConnectivityException, Exception):
            mat1 = [[1, 0, 0.0, 0],
                    [0, 1, 0., 0],
                    [0.0, 0.0, 1.0, 0],
                    [0.0, 0.0, 0.0, 1.0]]
        odom_lasttimestamp = self.odom_last_timestamp
        odom_timestamp = odom_2.header.stamp.secs
        self.dodmetry_time_difference = odom_lasttimestamp - odom_lasttimestamp
        odometry_position = [odom_2.pose.pose.position.x + self.robot1_initial_position.x,
                             odom_2.pose.pose.position.y + self.robot1_initial_position.y]
        odom_position = [self.odom_positio[0],odometry_position]
        self.odom_position = odom_position

        odom_speed_robot2 = [odom_2.twist.twist.linear.x, odom_2.twist.twist.linear.y, odom_2.twist.twist.angular.z, 0]
        odom_speed_robot2_tf = np.dot(mat1, odom_speed_robot2)
        if len(odom_speed_robot2_tf) != 0:
            odom_speed = [self.linear_speed[0],[odom_speed_robot2_tf[0], odom_speed_robot2_tf[1], odom_speed_robot2_tf[2]]]
        else:
            odom_speed = [self.linear_speed[0],[0, 0, 0, 0]]
        self.linear_speed = odom_speed
        self.odom_last_timestamp = odom_timestamp  # update the odometry time stamp

    def odometry_callback_3robots(self, odom_3):

        listener.waitForTransform(rospy.get_param("~map_link"), rospy.get_param("~robot3_odometry_link"), rospy.Time(0),
                                  rospy.Duration(0.1))
        t, r = listener.lookupTransform(rospy.get_param("~map_link"), rospy.get_param("~robot3_odometry_link"),
                                        rospy.Time(0))

        # for rosbag , at the beginning , tf data not received , for program running ,use try strcture,
        # if get error, just set tf as identity matrix

        try:
            t, r = listener.lookupTransform(rospy.get_param("~map_link"), rospy.get_param("~robot3_odometry_link"),
                                            rospy.Time(0))
            b = R.from_quat(r)
            angle = b.as_euler('zyx', degrees=False)

            mat1 = [[math.cos(angle[0]), -math.sin(angle[0]), 0.0, t[0]],
                    [math.sin(angle[0]), math.cos(angle[0]), 0., t[1]],
                    [0.0, 0.0, 1.0, t[2]],
                    [0.0, 0.0, 0.0, 1.0]]
        except(tf.LookupException, tf.ConnectivityException, Exception):
            mat1 = [[1, 0, 0.0, 0],
                    [0, 1, 0., 0],
                    [0.0, 0.0, 1.0, 0],
                    [0.0, 0.0, 0.0, 1.0]]
        odom_lasttimestamp = self.odom_last_timestamp
        odom_timestamp = odom_3.header.stamp.secs
        self.dodmetry_time_difference = odom_lasttimestamp - odom_lasttimestamp
        odometry_position = [odom_3.pose.pose.position.x + self.robot3_initial_position.x,
                             odom_3.pose.pose.position.y + self.robot3_initial_position.y]
        odom_position = [self.odom_positio[0],self.odom_positio[1], odometry_position]
        self.odom_position = odom_position

        odom_speed_robot3 = [odom_3.twist.twist.linear.x, odom_3.twist.twist.linear.y, odom_3.twist.twist.angular.z, 0]
        odom_speed_robot3_tf = np.dot(mat1, odom_speed_robot3)
        if len(odom_speed_robot3_tf) != 0:
            odom_speed = [self.linear_speed[0],self.linear_speed[1],
                          [odom_speed_robot3_tf[0], odom_speed_robot3_tf[1], odom_speed_robot3_tf[2]]]
        else:
            odom_speed = [self.linear_speed[0],self.linear_speed[1],[0, 0, 0, 0]]
        self.linear_speed = odom_speed
        self.odom_last_timestamp = odom_timestamp  # update the odometry time stamp

    def robot_identification(self):
        # compare teh calculated robots position with odometry position to identify the robots
        if len(self.calculated_position) != 0:
            correlation_matrix = np.zeros([len(self.calculated_position), len(self.odom_position)])

            # the distance between i-th in calculated and j-th object in [robot1,robot2,...robot_master]
            for i in range(np.size(correlation_matrix, 0)):
                for j in range(np.size(correlation_matrix, 1)):
                    correlation_matrix[i][j] = math.sqrt(
                        (self.calculated_position[i][0] - self.odom_position[j][0]) ** 2 + \
                        (self.calculated_position[i][1] - self.odom_position[j][1]) ** 2)

            # c_row[i]: index of minimal distance of i-th in calculate and c_row[i]-th in pre-defined sequence
            try:
                 c_row = np.argmin(correlation_matrix, axis=1)

            # c_column[j]: index of minimal distance of j-th in pre-defined sequence and c_column[j] in calculated
                 c_colunm = np.argmin(correlation_matrix, axis=0)
            except (Exception):
                c_colunm=[0]
                c_row=[0]
            label = c_colunm
            calculated_position = []

            for i in range(len(label)):
                print self.calculated_orientation
                print self.linear_speed
                if len(self.linear_speed) ==0:
                    self.linear_speed=[[0,0,0]]

                calculated_position_orientation_speed_element = [self.calculated_position[label[i]][0],
                                                                 self.calculated_position[label[i]][1], 0.15,
                                                                 self.calculated_orientation[i][0], self.calculated_orientation[i][1], self.calculated_orientation[i][2],
                                                                 self.calculated_orientation[i][3], self.linear_speed[i][0], self.linear_speed[i][1],
                                                                 self.linear_speed[i][2]]
                calculated_position.append(calculated_position_orientation_speed_element)
            self.calculated_position_rotation_speed = calculated_position

            rospy.loginfo("identified")
        else:
            label = [0]
            self.calculated_position_rotation_speed = []
            rospy.loginfo("robots hasn't been identified ")

    def update(self):
        while not rospy.is_shutdown():  # make sure update program keep running

            self.robot_identification()  # first identify the robots

            msg = PoseArray()
            msg.header.frame_id = rospy.get_param("~map_link")

            if len(self.calculated_position_rotation_speed) != 0:  # calculated_position is done

                if len(self.old_position_orientation_speed) == 0:  # the old position hasn't been updated
                    self.old_position_orientation_speed = self.calculated_position_rotation_speed  # directly use calculated_position update old_position
                    self.true_position = self.calculated_position_rotation_speed  # directly use calculated_position as true_position
                else:  # old position has been updated
                    self.true_position = []
                    true_position_sub = []
                    C = np.zeros([len(self.calculated_position_rotation_speed), len(self.old_position_orientation_speed)])

                    for i in range(np.size(C, 0)):
                        for j in range(np.size(C, 1)):
                            # C[i][j] means distance between i-th object in calculated position and j-th object
                            # in old position
                            C[i][j] = math.sqrt((self.old_position_orientation_speed[j][0] - self.calculated_position_rotation_speed[i][0]) ** 2 + \
                                                (self.old_position_orientation_speed[j][1] - self.calculated_position_rotation_speed[i][1]) ** 2)

                    n_row = np.argmin(C, axis=1)
                    n_column = np.argmin(C, axis=0)

                    if (n_row == n_column).all():
                        # no fusion and fission , no assignment error
                        for i in range(len(n_row)):

                            position_tracking_threshold = math.sqrt((self.linear_speed[i][0] * self.dodmetry_time_difference) ** 2 + (self.linear_speed[i][1] * self.dodmetry_time_difference) ** 2)

                            # consider 0.3 as static error (robot don't move)
                            ststic_tracking_threshold =0.5
                            if position_tracking_threshold < ststic_tracking_threshold:
                                position_tracking_threshold = 0.3
                            else:
                                position_tracking_threshold = ststic_tracking_threshold

                            if len(self.amcl_orientation) !=0:
                                amcl_orientation = R.from_quat(self.amcl_orientation)
                                amcl_orientation = amcl_orientation.as_euler('xyz', degrees=True)
                                calculated_orientation = R.from_quat(self.calculated_position_rotation_speed[i][3:7])
                                calculated_orientation = calculated_orientation.as_euler('xyz',degrees=True)
                                if abs(calculated_orientation[2]-amcl_orientation[2]) >90:
                                    correct_orientation = [0, 0, calculated_orientation[2]+180]
                                    correct_orientation = R.from_euler('xyz', correct_orientation, degrees=True)
                                    correct_orientation = correct_orientation.as_quat()
                                    self.calculated_position_rotation_speed[i][3:7] = correct_orientation

                            #position_tracking_threshold= 0.5     #short side of the robot
                            if C[i][i] < position_tracking_threshold:  # i-th object tracked
                                rospy.loginfo("position tracked")

                                true_position_sub.append(self.calculated_position_rotation_speed[i])  # update i-th tracked object

                            # here need fusion with  odometry data
                            else:  # i-th object untracked

                                rospy.loginfo("position untracked")

                                x = self.old_position_orientation_speed[i][0] + self.old_position_orientation_speed[i][7] * self.dodmetry_time_difference
                                y = self.old_position_orientation_speed[i][1] + self.old_position_orientation_speed[i][8] * self.dodmetry_time_difference
                                z = self.old_position_orientation_speed[i][2]
                                rotation_x = self.calculated_position_rotation_speed[i][3]
                                rotation_y = self.calculated_position_rotation_speed[i][4]
                                rotation_z = self.calculated_position_rotation_speed[i][5]
                                rotation_w = self.calculated_position_rotation_speed[i][6]
                                true_position_sub_element = [x, y, z, rotation_x, rotation_y, rotation_z, rotation_w, \
                                                             self.calculated_position_rotation_speed[i][7],
                                                             self.calculated_position_rotation_speed[i][8], \
                                                             self.calculated_position_rotation_speed[i][9]]

                                # update i-th untracked object with calculation by Odometry

                                true_position_sub.append(true_position_sub_element)

                        self.true_position = true_position_sub
                        for i in range(len(true_position_sub)):
                            msg_element = Pose()
                            msg_element.position.x = true_position_sub[i][0]
                            msg_element.position.y = true_position_sub[i][1]
                            msg_element.position.z = true_position_sub[i][2]
                            msg_element.orientation.x = true_position_sub[i][3]
                            msg_element.orientation.y = true_position_sub[i][4]
                            msg_element.orientation.z = true_position_sub[i][5]
                            msg_element.orientation.w = true_position_sub[i][6]
                            msg.poses.append(msg_element)

                        self.old_position_orientation_speed = true_position_sub
                        msg.header.stamp = rospy.Time.now()
                        self.pub.publish(msg)

                        self.loop_rate.sleep()
                    else:
                        compare = n_row == n_column
                        rospy.loginfo("assignment error")
                        for i in range(len(compare)):
                            if compare[i] == True:
                                true_position_sub.append(self.calculated_position_rotation_speed[i])
                            else:

                                x = self.old_position_orientation_speed[i][0] + self.calculated_position_rotation_speed[i][7] * self.dodmetry_time_difference
                                y = self.old_position_orientation_speed[i][1] + self.calculated_position_rotation_speed[i][8] * self.dodmetry_time_difference
                                z = self.old_position_orientation_speed[i][2]
                                rotation_x = self.calculated_position_rotation_speed[i][3]
                                rotation_y = self.calculated_position_rotation_speed[i][4]
                                rotation_z = self.calculated_position_rotation_speed[i][5]
                                rotation_w = self.calculated_position_rotation_speed[i][6]
                                true_position_sub_element = [x, y, z, rotation_x, rotation_y, rotation_z,
                                                             rotation_w]

                                # update i-th untracked object with calculation by Odometry

                                true_position_sub.append(true_position_sub_element)
                        for i in range(len(true_position_sub)):
                            msg_element = Pose()
                            msg_element.position.x = true_position_sub[i][0]
                            msg_element.position.y = true_position_sub[i][1]
                            msg_element.position.z = true_position_sub[i][2]
                            msg_element.orientation.x = true_position_sub[i][3]
                            msg_element.orientation.y = true_position_sub[i][4]
                            msg_element.orientation.z = true_position_sub[i][5]
                            msg_element.orientation.w = true_position_sub[i][6]
                            msg.poses.append(msg_element)
                            self.old_position_orientation_speed = true_position_sub
                            msg.header.stamp = rospy.Time.now()
                            self.pub.publish(msg)
                            self.loop_rate.sleep()
            else:  # no calculated_position data
                rospy.loginfo('position has not been calculated')
                self.loop_rate.sleep()


if __name__ == '__main__':
    rospy.init_node("laser_localization")

    # load a pre made map data


    map = np.loadtxt(rospy.get_param("~map_path"), float,delimiter=',')
    #map = np.loadtxt("/home/matchos/catkin_ws/src/laser_based_localization/map/gazebo_map.txt", float, delimiter=',')

    # define number of robots except master robot
    number_of_robots = rospy.get_param("~number_of_robots")
    listener = tf.TransformListener()
    track = Tracking()
    track.update()
    rospy.spin()