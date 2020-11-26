import math

import numpy as np




def find_angle(x, y, theta_step, initial_angle):  # input x, y list, return angle
    Var1 = []
    Var = []
    Var2 = []
    x = np.array(x)

    y = np.array(y)
    if initial_angle == 999:     # define a constant just to initial the program , no special meaning
        theta_0 = 0.0
        theta_stop = math.pi /2
    else:
        theta_0 = initial_angle - 2 * theta_step
        theta_stop = initial_angle + 2 * theta_step



    for theta in np.arange(theta_0, theta_stop, theta_step):
        C_1 = x * math.cos(theta) + y * math.sin(theta)  # the projection in direction e1
        C_2 = -x * math.sin(theta) + y * math.cos(theta)  # the projection in direction e2
        # calculate the max and min projection in two direction
        c_1_max = max(C_1)
        c_1_min = min(C_1)
        c_2_max = max(C_2)
        c_2_min = min(C_2)
        I_1 = []
        I_2 = []
        E_1 = []
        E_2 = []
        # d1, d2 : the minimal distance from each point to the two directions
        for i in range(len(x)):
            d_1 = min(c_1_max - C_1[i], C_1[i] - c_1_min)
            d_2 = min(c_2_max - C_2[i], C_2[i] - c_2_min)

            if d_1 < d_2:
                # the i-th point is closer to direction 1, so put i-th in I_1
                # E_1 is the list of distance of  first class points to edge 1
                I_1.append(i)
                E_1.append(d_1)
            else:
                # the i-th point is closer to direction 2, so put i-th in I_2
                I_2.append(i)
                E_2.append(d_2)

        E_1 = E_1
        E_2 = E_2

        # for each angle, calculate the variance of E_1,E_2, the best angle should be the minimal variance one

        var_1 = np.var(E_1)
        var_2 = np.var(E_2)
        var = var_1 + var_2
        Var1.append(var_1)
        Var2.append(var_2)
        Var.append(var)

    j = Var.index(min(Var))
    direction = theta_0 + j * theta_step
    return direction


def find_edge(x, y, a):  # input x, y and angle

    x = np.array(x)
    y = np.array(y)

    # the projection of point (x,y) in direction [cos(a),sin(a)] and [-sin(a),cos(a)]
    C_1 = x * math.cos(a) + y * math.sin(a)
    C_2 = -x * math.sin(a) + y * math.cos(a)

    a1 = math.cos(a)
    b1 = math.sin(a)
    a2 = -math.sin(a)
    b2 = math.cos(a)
    a3 = math.cos(a)
    b3 = math.sin(a)
    a4 = -math.sin(a)
    b4 = math.cos(a)

    c1 = min(C_1)
    c2 = min(C_2)
    c3 = max(C_1)
    c4 = max(C_2)

    # calculate the four peak points X1,X2,X3,X4
    A_1 = np.array([[a1, b1], [a2, b2]])  # lien 1 and line 2 intersection
    C_1 = np.array([[c1], [c2]])

    X1 = np.linalg.inv(A_1).dot(C_1)

    A_2 = np.array([[a1, b1], [a4, b4]])  # lien 1 and line 4 intersection
    C_2 = np.array([[c1], [c4]])
    X2 = np.linalg.inv(A_2).dot(C_2)

    A_3 = np.array([[a2, b2], [a3, b3]])  # lien 2 and line 3 intersection
    C_3 = np.array([[c2], [c3]])
    X3 = np.linalg.inv(A_3).dot(C_3)

    A_4 = np.array([[a3, b3], [a4, b4]])  # lien 3 and line 4 intersection
    C_4 = np.array([[c3], [c4]])
    X4 = np.linalg.inv(A_4).dot(C_4)

    # plt.scatter(X2[0], X2[1], marker='x')

    # find which point is the most reliable point
    X = [X1, X2, X3, X4]
    distance_realiable_point = []
    for i in range(len(X)):
        dis = sum(abs(x - X[i][0])) + sum(abs(y - X[i][1]))
        distance_realiable_point.append(dis)

    L = []
    H = []
    d_min = distance_realiable_point.index(min(distance_realiable_point))

    Peak_point = [X1, X2, X3, X4]
    # the rest 3 peak points
    Peak_point.pop(d_min)
    # print ('X=',X)

    distance = []
    for k in range(len(Peak_point)):
        dis = math.sqrt((Peak_point[k][0] - X[d_min][0]) ** 2 + (Peak_point[k][1] - X[d_min][1]) ** 2)
        distance.append(dis)
    # k_max is the index of across peak point
    k_max = distance.index(max(distance))
    # print New[k_max]

    d_max = [np.array_equal(Peak_point[k_max], point) for point in X].index(True)
    # print d_max

    # d_max = distance.index(max(distance))
    for h in range(len(X)):
        if h != d_min and h != d_max:
            # calculate the distance of other two points to the realiable point
            l1 = math.sqrt((X[d_min][0] - X[h][0]) ** 2 + (X[d_min][1] - X[h][1]) ** 2)
            L.append(l1)
            H.append(h)
        else:
            continue
    # size of pre-defined rectangular
    long_side = 0.9
    short_side = 0.58

    if (abs(long_side - L[0])) < (abs(long_side - L[1])) :
        X1 = X[d_min]
        X2 = X[H[0]]
        X3 = X[H[1]]
        X4 = X[d_max]
        X = [X1, X2, X3, X4]
        long_realibility = L[0]/long_side
        short_realibility = L[1]/short_side


    else:
        X1 = X[d_min]
        X2 = X[H[1]]
        X3 = X[H[0]]
        X4 = X[d_max]
        X = [X1, X2, X3, X4]
        long_realibility = L[1]/long_side
        short_realibility =L[0]/short_side


    realibility =min(long_realibility,short_realibility)

    realibility_threshold =0.3

    if realibility>realibility_threshold:
        alpha2 = math.atan2((X2[1] - X1[1]), (X2[0] - X1[0]))
        X2 = [X1[0] + long_side * math.cos(alpha2), X1[1] + long_side * math.sin(alpha2)]
        alpha3 = math.atan2((X3[1] - X1[1]), (X3[0] - X1[0]))
        X3 = [X1[0] + short_side * math.cos(alpha3), X1[1] + short_side * math.sin(alpha3)]
        X4 = [X2[0] + X3[0] - X1[0], X2[1] + X3[1] - X1[1]]

        center = [0.5 * (X1[0] + X4[0]), 0.5 * (X1[1] + X4[1])]  # center point of the rectangle
        return center, realibility
    else:
        alpha2 = math.atan2((X2[1] - X1[1]), (X2[0] - X1[0]))
        X2 = [X1[0] + long_side * math.cos(alpha2), X1[1] + long_side * math.sin(alpha2)]
        alpha3 = math.atan2((X3[1] - X1[1]), (X3[0] - X1[0]))
        X3 = [X1[0] + short_side * math.cos(alpha3), X1[1] + short_side * math.sin(alpha3)]
        X3_2 = [X1[0] - short_side * math.cos(alpha3), X1[1] - short_side * math.sin(alpha3)]

        X4 = [X2[0] + X3[0] - X1[0], X2[1] + X3[1] - X1[1]]
        X4_2 = [X2[0] + X3_2[0] - X1[0], X2[1] + X3_2[1] - X1[1]]

        center_1 = [0.5 * (X1[0] + X4[0]), 0.5 * (X1[1] + X4[1])]
        center_2 = [0.5 * (X1[0] + X4_2[0]), 0.5 * (X1[1] + X4_2[1])]
        center=[center_1,center_2]
        return center,realibility







def rectangular(x, y):
    theta_step1 = (10.0 / 180) * math.pi
    theta_step2 = (5.0 / 180) * math.pi
    theta_step3 = (2.0 / 180) * math.pi
    theta_step4 = (0.5 / 180) * math.pi
    a1 = find_angle(x, y, theta_step1, initial_angle=999)  # 999 is just a number to do the initial step ,see line 15
    a2 = find_angle(x, y, theta_step2, initial_angle=a1)
    a3 = find_angle(x, y, theta_step3, initial_angle=a2)
    a4 = find_angle(x, y, theta_step4, initial_angle=a3)
    c,p = find_edge(x, y, a4)
    return a4, c ,p
