#!/usr/bin/env python

# This script is mainly DBSCAN clustering,
# inputs are fused laser data, out put are clustered groups and the number of groups

from sklearn.cluster import DBSCAN
import numpy as np


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


def get_index(lst=None, item=''):
    return [i for i in range(len(lst)) if lst[i] == item]


def cluster(x, y):
    group = []
    points = np.zeros(shape=(len(x), 2))
    for i in range(len(x)):
        points[i] = [x[i], y[i]]
    # Here eps and min_samples are two important parameter for DBSCAN,
    # it can change the cluster result , should be correct choosen
    clustering = DBSCAN(eps=0.4, min_samples=20).fit(points)
    noise = get_index(clustering.labels_, -1)     # label -1 is noise
    if noise != []:
        points = np.delete(points, noise, axis=0)
        label = np.array(clustering.labels_)
        label = np.delete(label, noise, axis=0)
        clustering.labels_ = label.tolist()
    setl = set(clustering.labels_)
    number = len(setl)
    new_X = []
    new_Y = []
    for j in range(number):
        array_x = []
        array_y = []
        for k in range(len(clustering.labels_)):
            if clustering.labels_[k] == j:
                array_x.append(points[k][0])
                array_y.append(points[k][1])
            else:
                continue
        new_X.append(array_x)
        new_Y.append(array_y)
    for g in range(len(new_X)):
        gro = Points(g, new_X[g], new_Y[g])
        group.append(gro)
    return group, number

