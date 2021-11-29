#!/usr/bin/env python3
import rospy
import ros_numpy
import numpy as np
import std_msgs.msg
from sensor_msgs.msg import PointCloud2,PointField
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Pose
import ros_numpy
import pcl

import grasp


def cloud_callBack(cloud_msg):
    global cloud_state
    cloud_state = cloud_msg

def pose_callBack(pose_msg):
    global pose_state
    pose_state = pose_msg

def main():

    rospy.init_node('listener', anonymous=True)
    sub_cloud = rospy.Subscriber("/cloud_segmented", PointCloud2, cloud_callBack)
    sub_pose = rospy.Subscriber("/obj_pose", Pose, pose_callBack)

    pub_grasp = rospy.Publisher('/grasp_region', PointCloud2, queue_size=10)
    pub_geometry = rospy.Publisher('/grasp_type', std_msgs.msg.Int16, queue_size=10)
    g_w = rospy.get_param("/gripper_width")
    g_h = rospy.get_param("/gripper_height")
    n = rospy.get_param("/n")
    frame = rospy.get_param("/frame")

    while not rospy.is_shutdown():
        rate = rospy.Rate(10)
        cloud_global = "cloud_state" in globals()
        pose_global = "pose_state" in globals()
        if cloud_global and pose_global:
            pc = ros_numpy.numpify(cloud_state)
            points=np.zeros((pc.shape[0],3))
            points[:,0]=pc['x']
            points[:,1]=pc['y']
            points[:,2]=pc['z']

            cloud = pcl.PointCloud()
            cloud.from_array(points.astype('float32'))
            indice, cloud_grasp = grasp.grasp(cloud,g_h,g_w,n)
            if indice == -2:
                print("No possible grasp")
                exit(0)

            cloud_array = np.asarray(cloud_grasp)
            header = std_msgs.msg.Header()
            header.stamp = rospy.Time.now()
            header.frame_id = frame
            cloud_out = pc2.create_cloud_xyz32(header, cloud_array)
            pub_grasp.publish(cloud_out)
            pub_geometry.publish(indice)

        rate.sleep()


if __name__ == '__main__':
    
	main()