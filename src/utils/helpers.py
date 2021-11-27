import numpy as np
import rospy
import math
from pathlib import Path

def euclidean_distance(a, b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 +(a[2]-b[2])**2)

def pc_transform(pc, translation):
    pointcloud = np.dot(translation, np.c_[
                            pc, np.ones(len(pc))].T).T
    pointcloud = np.delete(pointcloud, 3, 1)
    return pointcloud

def depth_to_pc(depth_img, save_to_file: bool = False):

    W = depth_img.shape[1]
    H = depth_img.shape[0]
    FOV = 90
    Fx = Fy = W / (2 * np.tan(FOV * np.pi / 360))
    Cx = W / 2
    Cy = H / 2
    # cameraToDrone = np.array([
    #     [0.0000463,  0.0000000, 1.0000000, 0.5],
    #     [-1.0000000,  0.0000463, 0.0000463, 0.0],
    #     [-0.0000463, -1.0000000, 0.0000000, 0.1],
    #     [0.0000000,  0.0000000, 0.0000000, 1.0]
    # ])

    def depthConversion(PointDepth, f):
        H = PointDepth.shape[0]
        W = PointDepth.shape[1]
        i_c = float(H) / 2 - 1
        j_c = float(W) / 2 - 1
        columns, rows = np.meshgrid(np.linspace(
            0, W-1, num=W), np.linspace(0, H-1, num=H))
        DistanceFromCenter = ((rows - i_c)**2 + (columns - j_c)**2)**(0.5)
        PlaneDepth = PointDepth / (1 + (DistanceFromCenter / f)**2)**(0.5)
        return PlaneDepth

    def generatepointcloud(depth):
        rows, cols = depth.shape
        c, r = np.meshgrid(np.arange(cols), np.arange(rows), sparse=True)
        valid = (depth > 0) & (depth < 255)
        z = 1000 * np.where(valid, depth / 256.0, np.nan) / 3.9216 # пробовать подобрать.
        x = np.where(valid, z * (c - Cx) / Fx, 0)
        y = np.where(valid, z * (r - Cy) / Fy, 0)
        return np.dstack((x, y, z))

    if (depth_img is None):
        rospy.logwarn(
            "Camera is not returning image, please check airsim for error messages")
        return None
    else:
        image3d = generatepointcloud(depthConversion(depth_img, Fx))
        pointcloud = np.reshape(
            image3d, (image3d.shape[0]*image3d.shape[1], image3d.shape[2]))
        # pc_transform(pointcloud, cameraToDrone)
        if save_to_file:
            rgb = "%d %d %d" % (0, 255, 0)
            pth = str(Path.home()) + "/iris1pc" + \
                str(rospy.get_time()) + ".asc"
            f = open(pth, "w")
            rospy.logwarn("Writing file %s" % pth)
            for pt in pointcloud:
                f.write("%f %f %f %s\n" % (pt[0], pt[1], pt[2], rgb))
            rospy.logwarn("File saved")
            f.close()
        return image3d, pointcloud


class Node:
    """ Vertex of a graph with its parent and cost """
    
    def __init__(self, parent=None, position = None) -> None:
        self.parent = parent
        self.position = position

        # Costs
        self.g = 0.0
        self.h = 0.0
        self.f = 0.0

    def __eq__(self, other):
        if np.array_equal(self.position, other.position):
            return True
        else:
            return False

class Area:
    """ Limiting factor for path planning algorithms """
    
    def __init__(
        self,
        shift = np.array([0, 0, 0]),
        min_x: float = 0, max_x: float = 40,
        min_y: float = -20, max_y: float = 20,
        min_z: float = -0.1, max_z: float = 19
    ) -> None:
        self.min_vector = np.array([min_x, min_y, min_z]) + shift
        self.max_vector = np.array([max_x, max_y, max_z]) + shift
    
    def get_vectors(self):
        return self.min_vector, self.max_vector

    def within_area(self, point):
        if ((point >= self.min_vector).all() or
            (point <= self.max_vector).all()):
            return True
        return False

class NPPose:
    """ NumPy based pose"""

    def __init__(self, ros_pose):
        self.position = np.array([
            ros_pose.position.x, 
            ros_pose.position.y, 
            ros_pose.position.z
        ])
        self.orientation = np.array([
            ros_pose.orientation.x,
            ros_pose.orientation.y,
            ros_pose.orientation.z,
            ros_pose.orientation.w
        ])

