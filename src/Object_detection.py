import rospy
import math
import numpy
from  matplotlib import pyplot as plt
from scipy.optimize import leastsq

def err(p, x, y):
    return p[0] * x + p[1] - y


def object_detection(group, number_of_objects):
    ret=[]

    for i in range(number_of_objects):
        line_points=[]
        data_x = group[i].x
        data_y = group[i].y
        length = len(group[i].x)
        x_0 = group[i].x[0]
        y_0 = group[i].y[0]
        x_last = group[i].x[length - 1]
        y_last = group[i].y[length - 1]

        # define line y=kx+b
        k = (y_last - y_0) / (x_last - x_0)
        b = ((x_last * y_0) - (x_0 * y_last)) / (x_last - x_0)
        d = []
        for j in range(length):
            d_j = (abs(k * data_x[j] - data_y[j] + b)) / math.sqrt(k * k + 1)
            d.append(d_j)

        extrem_point_index = d.index(max(d))

        extrem_point = [data_x[extrem_point_index],data_y[extrem_point_index]]
        group[i].extreme_point=extrem_point
        #least square to find two lines
        line0=[data_x[0:(extrem_point_index+1)],data_y[0:(extrem_point_index+1)]]
        line1=[data_x[extrem_point_index:(length+1)],data_y[extrem_point_index:(length+1)]]
        line0_x=numpy.array(line0[0])
        line0_y=numpy.array(line0[1])
        line1_x=numpy.array(line1[0])
        line1_y=numpy.array(line1[1])

        p0 = [10, 20]
        ret_0 = leastsq(err, p0, args=(line0_x,line0_y))
        ret_1 = leastsq(err, p0, args=(line1_x,line1_y))

        x0 = numpy.array([data_x[0],data_x[extrem_point_index]])
        y0= ret_0[0][0] *x0 + ret_0[0][1]

        x1 =numpy.array([data_x[extrem_point_index],data_x[length-1]])
        y1 =ret_1[0][0]*x1 +ret_1[0][1]

        line_points.append(x0)
        line_points.append(y0)
        line_points.append(x1)
        line_points.append(y1)
        group[i].extrem_point = extrem_point

        ret.append(line_points)



        # print ('x0=',x0,'y0=',y0)
        #
        # plt.plot(x0,y0)
        # # plt.plot(x1,y1)
        # plt.show()
        # print (extrem_point)
        # group[i].extrem_point = extrem_point
        # ret.append(ret_0)
        # ret.append(ret_1)

    return group,ret






