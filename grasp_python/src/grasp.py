# -*- coding: utf-8 -*-
from __future__ import print_function

import numpy as np
import pcl
import grasp_filters
import grasp_helper
from obj_parts import ObjParts
import operator


def grasp(cloud, g_h = 0.04, g_w = 0.14, n = 6):

    """
        Estimate the best grasp and return the best grasp location.

        Parameters:
        -----------
            cloud : pcl.PointCloud()
            g_h: gripper height in meters
            g_w: gripper width in meters
            n: number of points to calculate curvature
        Returns:
        -----------
            indice: The indice of the best region to grasp
            cloud : pcl.PointCloud()
    """

    parts = []
    obj_parts= []
    is_simple = False
    indice = -1

    #cloud = pcl.load('/home/daniel/pcd256/clamp.pcd')
    #file_pre = "/home/daniel/python_tutorial/pcl/results/part"
    #file_pos = ".pcd"
    min = np.min(cloud, axis=0)
    max = np.max(cloud, axis=0)
    altura = max[1] - min[1]
    largura = max[0] - min[0]
    print("Height: ", altura)
    print("Width: ", largura)
    parts_temp = grasp_filters.crop_height(min,max,g_h,cloud)
    size_parts_temp = len(parts_temp)
    print("Number of parts: ", size_parts_temp)

    for i in range(size_parts_temp):
        cluster = grasp_filters.get_clusters(parts_temp[i])
        for j in range(len(cluster)):
            parts.append(cluster[j])

    #parts = parts_temp
    size_parts = len(parts)
    print("Number of clusters: ", size_parts)
    if size_parts == 0:
        parts = parts_temp

    if parts == 0:
        print("Grasp in the center")
    else:
        for i in range(len(parts)):
            print("Part Size: ", parts[i].size)
            #file_save = file_pre + str(i) + file_pos
            #print("Salvo como: ", file_save)
            #pcl.save(parts[i], file_save)

            min = np.min(parts[i], axis=0)
            max = np.max(parts[i], axis=0)
            altura = max[1] - min[1]
            largura = max[0] - min[0]

            N = parts[i].size/n
            r = N*2
            curv= grasp_filters.get_curvature_points(parts[i],r)
            delta = grasp_filters.get_geometry(parts[i],N)
            part = ObjParts(parts[i],altura,largura,curv, delta,i)
            obj_parts.append(part)
    
    obj_parts.sort(key=operator.attrgetter('curvatura'))

    for i in range(len(obj_parts)):
        print(obj_parts[i])

    if grasp_helper.check_simple_geometry(obj_parts):
        print("Grasp Object in the center")
        is_simple = True
    else:
        indice = grasp_helper.get_grasp(obj_parts, g_w)


    print("Best part to grasp: ", indice) 

    if indice == -2:
        return -2,0
    elif indice == -1:
        return -1, cloud
    else:
        return indice, obj_parts[indice].cloud
    
    #pcl.save(cloud, "/home/daniel/python_tutorial/pcl/results/cloud.pcd")
