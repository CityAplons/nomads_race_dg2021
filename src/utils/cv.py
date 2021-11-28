import numpy as np
import rospy
import cv2

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
        if save_to_file:
            rgb = "%d %d %d" % (0, 255, 0)
            pth = "./iris1pc" + str(rospy.get_time()) + ".asc"
            f = open(pth, "w")
            rospy.logwarn("Writing file %s" % pth)
            for pt in pointcloud:
                f.write("%f %f %f %s\n" % (pt[0], pt[1], pt[2], rgb))
            rospy.logwarn("File saved")
            f.close()
        return image3d, pointcloud