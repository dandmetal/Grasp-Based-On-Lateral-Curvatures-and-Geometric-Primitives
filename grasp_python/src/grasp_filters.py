import pcl
import numpy as np
import math
from scipy.spatial.transform import Rotation as R

def rotationMatrixToEulerAngles(R) :
    """
    Convert rotation matrix to euler.

    Parameters:
    -----------
        R: Rotation matrix 3x3
    Returns:
    -----------
        Euler: np.array containing euler vector 3x1
    """
    
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    singular = sy < 1e-6
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
    return np.array([x, y, z])


def rotate_cloud(cloud, orientation):
    """
    Rotate the cloud to the designed orientation.

    Parameters:
    -----------
        cloud: np.array 3xN
        orientation: np array 3x1
    Returns:
    -----------
        cloud_normi: np.array 3xN
    """
    rotation = R.from_rotvec([0,0,orientation[2]] )
    cloud_rotation = rotation.apply(cloud)
    rotation = R.from_rotvec([orientation[1],0,0] )
    cloud_rotation = rotation.apply(cloud_rotation)
    rotation = R.from_rotvec([0,orientation[2],0] )
    cloud_rotation = rotation.apply(cloud_rotation)
    cloud_normi = pcl.PointCloud()
    cloud_normi.from_array(cloud_rotation.astype('float32'))

    return cloud_normi

def paint_cloud(cloud, rgb):
    """
    Transform a point xyz point cloud in xyzrgb.

    Parameters:
    -----------
        cloud: pcl.PointCloud()
        rgb: np array 3x1
    Returns:
    -----------
        cloud_color: pcl.PointCloud_PointXYZRGB()
    """

    cloud_color = pcl.PointCloud_PointXYZRGB()
    array = np.asarray(cloud)
    a = np.empty([cloud.size, 4])
    r = rgb[0]
    g = rgb[1]
    b = rgb[2]
    for i in range(len(a)):
        a[i][0] = array[i][0]
        a[i][1] = array[i][1]
        a[i][2] = array[i][2]
        a[i][3] = r << 16 | g << 8 | b
    cloud_color.from_array(a.astype('float32'))
    return cloud_color

def get_curvature(cloud, indices):
    """
    Get the curvature of a region using PCA.

    Parameters:
    -----------
        cloud: pcl.PointCloud()
        indicies: np array N
    Returns:
    -----------
        curvature: float
    """

    points = np.asarray(cloud)
    M = np.array([ points[i] for i in indices[0] ]).T
    M = np.cov(M)
    
    # eigen decomposition
    V, E = np.linalg.eig(M)
    # h3 < h2 < h1
    h1, h2, h3= V

    curvature = h3 / (h1 + h2 + h3)
    return curvature

def get_cloud_orientation(cloud):
    """
    Get the orientation using PCA.

    Parameters:
    -----------
        cloud: pcl.PointCloud()
    Returns:
    -----------
        rot: array 3x1
    """
    points = np.asarray(cloud)
    M = points.T
    M = np.cov(M)
    V, E = np.linalg.eig(M)
    rot = rotationMatrixToEulerAngles(E)

    return rot

def crop_height(min, max, gripper_height, cloud):
    """
    Crop the cloud in smaller objects

    Parameters:
    -----------
        min: np array 3x1
        max: np array 3x1
        gripper_height: float
        cloud: pcl.PointCloud()
    Returns:
    -----------
        parts_cloud: array N
    """
    altura = max[1] - min[1]
    if gripper_height >= altura:
        return 0
    num_parts = altura/gripper_height
    act_min = max[1]
    next_min = act_min - gripper_height
    #act_min = act_min - gripper_height * 0.15
    parts_cloud = []
    for i in range(int(num_parts + 1)):
        passthrough = cloud.make_passthrough_filter()
        passthrough.set_filter_field_name("y")
        passthrough.set_filter_limits(next_min, act_min)
        cloud_filtered = passthrough.filter()
        if cloud_filtered.size > 8 and (act_min - next_min >= (gripper_height - 0.01)):
            parts_cloud.append(cloud_filtered)
        act_min = next_min
        next_min = next_min - gripper_height
    return parts_cloud

def get_curvature_points(cloud, radius): 
    """
    Get the curvature around the laterals of the object.

    Parameters:
    -----------
        cloud: pcl.PointCloud()
        radius: int
    Returns:
    -----------
        curvature: float
    """
    #size = cloud.size
    kd = cloud.make_kdtree_flann()
    if radius <= 3:
        radius = 4
    #min = np.min(cloud, axis=0)
    #max = np.max(cloud, axis=0)
    arg_min = np.argmin(cloud, axis=0)
    arg_max = np.argmax(cloud, axis=0)
    searchPoint = cloud[arg_min[0]]
    searchPoint2 = cloud[arg_max[0]]
    points_1 = np.array([[searchPoint[0], searchPoint[1], searchPoint[2]]], dtype=np.float32)
    points_2 = np.array([[searchPoint2[0], searchPoint2[1], searchPoint2[2]]], dtype=np.float32)
    
    pc_1 = pcl.PointCloud()
    pc_1.from_array(points_1)
    pc_2 = pcl.PointCloud()
    pc_2.from_array(points_2)
    indices, sqr_distances = kd.nearest_k_search_for_cloud(pc_1, radius)
    indices2, sqr_distances2 = kd.nearest_k_search_for_cloud(pc_2, radius)

    curv1 = get_curvature(cloud,indices)
    curv2 = get_curvature(cloud,indices2)

    return curv1 + curv2

def get_geometry(cloud, radius):
    """
    Get the geometry of the object.

    Parameters:
    -----------
        cloud: pcl.PointCloud()
        radius: int
    Returns:
    -----------
        curvature: float
    """
    kd = cloud.make_kdtree_flann()
    total = 0
    total_curv = 0
    if radius <= 2:
        radius = 3
    for i in range(cloud.size):
        searchPoint = cloud[i]
        points = np.array([[searchPoint[0], searchPoint[1], searchPoint[2]]], dtype=np.float32)
        pc= pcl.PointCloud()
        pc.from_array(points)
        indices, sqr_distances = kd.nearest_k_search_for_cloud(pc, radius)
        curv = get_curvature(cloud,indices)
        total = total + 1
        if curv <= 0.01:
            total_curv = total_curv + 1
    return float(total_curv)/float(total)

def get_clusters(cloud_filtered):
    """
    Get clusters from the cloud.

    Parameters:
    -----------
        cloud: pcl.PointCloud()
    Returns:
    -----------
        clusters: pcl.PointCloud() array N
    """
    clusters = []
    tree = cloud_filtered.make_kdtree()


    ec = cloud_filtered.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance (0.03)
    ec.set_MinClusterSize (5)
    ec.set_MaxClusterSize (100)
    ec.set_SearchMethod (tree)
    cluster_indices = ec.Extract()
    cloud_cluster = pcl.PointCloud()

    for j, indices in enumerate(cluster_indices):
        #print('indices = ' + str(len(indices)))
        points = np.zeros((len(indices), 3), dtype=np.float32)
        cloud_cluster = pcl.PointCloud()

        for i, indice in enumerate(indices):
            points[i][0] = cloud_filtered[indice][0]
            points[i][1] = cloud_filtered[indice][1]
            points[i][2] = cloud_filtered[indice][2]
        cloud_cluster.from_array(points)
        clusters.append(cloud_cluster)
    return clusters