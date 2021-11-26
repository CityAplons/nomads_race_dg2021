import sys
import cv2
import time
import open3d as o3d
import rospy
import airsim
import numpy as np
from threading import Thread
from scipy import signal
from scipy.spatial import KDTree
from scipy.spatial.transform import Rotation as R

from utils.controller import dronePositionController
from utils.volumetric_rrt import RRTStar
from utils.helpers import *

sys.setrecursionlimit(1500)
freq = 30

# to calculate the error definition
# up left x, y, z, w, h,
w1 = [[50.0, 3.0, 5.0, 6.0, 2.0], [50.0, 1.0, 2.0, 2.0, 6.0]]


def race_init(count: int, pid_params: dict = None):
    rospy.loginfo("Nikita's RACE node started")
    formation = RaceFormation(count, pid_params)
    rate = rospy.Rate(freq)
    while not rospy.is_shutdown():
        formation.loop()
        rate.sleep()
    for inst in formation.instances:
        inst["controller"].set_arm(False)


class RaceFormation:
    def __init__(self, num, pid_params: dict = None) -> None:
        rospy.on_shutdown(self.on_shutdown_cb)

        self.num = num
        self.airsim_client = airsim.MultirotorClient()
        self.airsim_client.confirmConnection()

        # show camera parameters
        distort_depth = self.airsim_client.simGetDistortionParams(airsim.ImageType.DepthPlanar)
        ProjectionMatrix = self.airsim_client.simGetCameraInfo(airsim.ImageType.DepthPlanar).proj_mat
        rotationMatrix = self.airsim_client.simGetCameraInfo(airsim.ImageType.DepthPlanar).pose.orientation
        cameraPosition = self.airsim_client.simGetCameraInfo(airsim.ImageType.DepthPlanar).pose.position
        print('-----distort_depth', distort_depth)
        print('-----ProjectionMatrix', ProjectionMatrix)
        print('-----rotationMatrix', rotationMatrix)
        print('-----cameraPosition', cameraPosition)

        self.run_cv = True
        self.target = np.array([10., 0., 3.])

        self.previous_pose = None

        self.run_cv_task = False
        self.path_found = False

        # ---for holes detection---
        self.current_holes = []
        self.poses_in_hole = []
        self.is_hole_detected = False
        self.min_size_drone = [1.95, 1.95]  # Y, Z metes for one drone
        self.check_angle = [-90, 90]
        self.check_id = 0
        # -------

        self.instances = []
        for id, vehicle in enumerate(self.airsim_client.listVehicles(), 1):
            rospy.loginfo("Airsim instance %d: %s", num, vehicle)
            first_point = np.array([20, 0, (12. - 2*id)])
            instance = {
                "name": vehicle,
                "id": id,
                "controller": dronePositionController(id, pid_params, pub_rate=freq),
                "waypoints": [first_point]
            }
            if(id == 1):
                self.cv_task = Thread(target=self.cv_thread, args=(instance,))
                self.cv_task.start()
            self.instances.append(instance)

        self.tat = 0
        self.freeze_pos = None
        airsim.wait_key('Press any key to execute RACE task')

    def on_shutdown_cb(self):
        self.run_cv = False
        time.sleep(0.5)
        del self
        rospy.logfatal("shutdown")

    def __del__(self):
        self.cv_task.join()

    def loop(self) -> None:

        self.arrived_num = 0
        formation_center_pose = np.array([0, 0, 0], dtype=float)

        for instance in self.instances:
            # Takeoff
            if not instance["controller"].is_armed:
                instance["controller"].set_arm(True)
                while not instance["controller"].takeoff():
                    continue

            # If copter in the arrival radius, set to arrived
            # move to next point in list
            if instance["controller"].is_arrived:
                self.arrived_num += 1

                if len(instance["waypoints"]):
                    rospy.loginfo("Following waypoint")
                    self.target = instance["waypoints"].pop(0)
                    instance["controller"].set_position(self.target)
                elif not self.run_cv_task:
                    instance["controller"].set_yaw(30)
                    print('get global pose',
                          instance["controller"].get_global_position())
                    self.previous_pose = instance["controller"].get_global_position(
                    )
                    self.cv_request_path(
                        instance["controller"].get_global_position() + np.array([30., 0., 0.]))
            # Re-calculate center pose location
            formation_center_pose += instance["controller"].get_global_position() / \
                self.num

        self.formation_center_pose = formation_center_pose

    def cv_request_path(self, point):
        self.target = point
        self.run_cv_task = True

    def cv_thread(self, instance: dict):

        rospy.loginfo("CV task created")
        max_found = False
        while self.run_cv:
            if self.run_cv_task:
                responses = self.airsim_client.simGetImages([
                    airsim.ImageRequest(
                        '0', airsim.ImageType.Scene, compress=False),
                    airsim.ImageRequest('1', airsim.ImageType.DepthPerspective, True, False)], vehicle_name=instance["name"])
                img1d = np.array(responses[1].image_data_float, dtype=np.float)
                img1d[img1d > 255] = 255
                img2d = np.reshape(
                    img1d, (responses[1].height, responses[1].width))
                img_pc, drone_pc = depth_to_pc(img2d)

                
                cameraToDrone = np.array([
                    [0.0000328, 0.0000000, 1.0000000],
                    [1.0000000, 0.0000328, -0.0000328],
                    [-0.0000328, 1.0000000, 0.0000000],
                ])

                dronePos = np.array(
                    [instance["controller"].get_global_position()])
                droneRot = R.from_quat(
                    instance["controller"].get_global_orientation()).as_matrix()
                droneRot = np.dot(droneRot, cameraToDrone)
                droneToWorld = np.concatenate((droneRot, dronePos.T), axis=1)
                droneToWorld = np.concatenate(
                    (droneToWorld, np.array([[0., 0., 0., 1.]])))

                wall_roi = [30, responses[1].height - 40, 75,
                            responses[1].width - 75]  # x1, x2, y1, y2
                # wall_roi = [30, responses[1].height - 30, 80, responses[1].width - 80 ]
                wall_pc = img_pc[wall_roi[0]:wall_roi[1],
                                 wall_roi[2]:wall_roi[3]]
                wall_pc = np.reshape(
                    wall_pc, (wall_pc.shape[0]*wall_pc.shape[1], wall_pc.shape[2]))
                wall_pc = pc_transform(wall_pc, droneToWorld)
                delta = np.abs(np.min(wall_pc[:, 0]) - dronePos[0, 0])

                # moving to wall
                if delta > 10.0:
                    rospy.logwarn("%f", delta)
                    point = dronePos[0] + np.array([1.0, 0, 0])
                    point[1] = self.previous_pose[1]
                    point[2] = self.previous_pose[2]
                    self.previous_pose = point
                    instance["controller"].set_position(point)

                if delta <= 10.0:
                    # delay to stabilaze drone
                    time.sleep(10.0)
                    # --- holes detection and poses calculation ---
                    depth = img1d.reshape(
                        responses[1].height, responses[1].width)
                    depth = np.array(depth, dtype=np.uint8)

                    self.depth_hole_detection(
                        depth, droneToWorld, img_pc, th=20)

                    if self.is_hole_detected:
                        self.get_poses_in_holes()
                        print(self.poses_in_hole[0][0])
                        print('go to ', self.poses_in_hole[0][0])
                        print("drone_pos", instance["controller"].get_global_position())

                    if not self.is_hole_detected:
                        instance["controller"].set_yaw(
                            30 + self.check_angle[self.check_id])
                        self.check_id += 1

                if self.is_hole_detected:

                    # instance["controller"].set_position(
                    #     self.poses_in_hole[0][0])
                    # while self.achieved_point(self.poses_in_hole[0][0], instance["controller"].get_global_position(), 0.1):
                    #     pass

                    # next_point = self.poses_in_hole[0][0] + \
                    #     np.array([5.0, 0, 0])
                    # print('next_point', next_point)
                    # instance["controller"].set_position(next_point)
                    # while self.achieved_point(next_point, instance["controller"].get_global_position(), 0.1):
                    #     pass
                    # rospy.logwarn("end wall, next one")

                    self.current_holes = []
                    self.poses_in_hole = []
                    self.run_cv = True
                    self.check_id = 0

                # world_pc = pc_transform(drone_pc, droneToWorld)
                # roi = world_pc - dronePos[0]
                # trimed_ids = np.argwhere(
                #     (roi[:, 2] > 1)&
                #     (roi[:, 2] < 18)&
                #     (roi[:, 0] < 40)&
                #     (roi[:, 1] < 40))
                # max_pos = np.amax(roi[trimed_ids], axis=0)[0]
                # #target = self.target
                # target = max_pos
                # self.target = target
                # target_yaw = np.arctan2(target[1], target[0])
                # print(target, target_yaw)

                # instance["controller"].set_yaw(target_yaw*2*np.pi)
                # instance["controller"].set_position(instance["controller"].get_global_position())
                # time.sleep(3)

                # pcd = o3d.geometry.PointCloud()
                # pcd.points = o3d.utility.Vector3dVector(world_pc)
                #  # o3d.io.write_point_cloud("/home/nikita/pc" + str(rospy.get_time()) + ".xyz", pcd)
                # voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(
                #     pcd,
                #     voxel_size=0.6
                # )
                # # o3d.visualization.draw_geometries([voxel_grid])
                # #path_planner = AStar(voxel_grid)

                # cur_position = instance["controller"].get_global_position()
                # shifted = np.array([cur_position[0], cur_position[1], 0])
                # search_volume = Area(shift=shifted, min_y=-40, max_y=40)
                # if search_volume.within_area(target):
                #     path_planner = RRTStar(
                #         dronePos[0], voxel_grid, 7, search_volume, resolution=20, it_limit=1500)
                #     path = path_planner.search(target)
                #     if path is not None:
                #         #path = path_planner.smooth_path(path)
                #         instance["waypoints"] = path[1:]
                #         self.path_found = True
                #         self.run_cv_task = False
                # else:
                #     rospy.logwarn("Target position out of space")
                #     time.sleep(0.5)
            else:
                time.sleep(0.5)
            time.sleep(0.03)

    def achieved_point(self, tp, cp, tol):
        achieved = False
        # print(np.linalg.norm(tp - cp))
        if np.linalg.norm(tp - cp) > tol:
            achieved = True
        return achieved

    def depth_hole_detection(self, depth, droneToWorld, img_pc, th=20):
        (T, thresh) = cv2.threshold(
            depth, th, 255, cv2.THRESH_BINARY_INV)
        contours, hierarchy = cv2.findContours(
            thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        cv2.imwrite('/home/asad/thresh.png', thresh)

        k = 0
        print('find counters n =', len(contours))

        # if drone detected counters
        if len(contours) > 1:
            self.is_hole_detected = True

        if len(contours) <= 1:
            self.is_hole_detected = False

        if self.is_hole_detected:
            k = 0
            for contour in contours:
                if k > 0:
                    print('contour', k)
                    box = cv2.boundingRect(contour)
                    print(box[0], box[1], box[2], box[3])
                    # border_point1 = img_pc[box[1], box[0]]
                    # border_point2 = img_pc[box[1] + box[3], box[0]+box[2]]
                    if box[0] == 144:
                        box[0] -= 1
                    if box[1] == 256:
                        box[1] -= 1
                    l = 0
                    if box[1] + box[3] == 144:
                        l = -1
                    if box[0] + box[2] == 256:
                        l = -1
                    border_point1 = img_pc[box[1] - 1, box[0]-1]
                    border_point2 = img_pc[box[1] +
                                           box[3] + l, box[0] + box[2] + l]
                    border_points = pc_transform(
                        [border_point1, border_point2], droneToWorld)
                    border_point1 = border_points[0]
                    border_point2 = border_points[1]
                    print("border_points:", border_point1, border_point2)
                    # current_holes: x_wall, y_left_up, z_left_up, w, h, 
                    w = abs(border_point2[1] - border_point1[1])
                    h = abs(border_point2[2] - border_point1[2])
                    # w = box[2] * 0.08
                    # h = box[3] * 0.085714

                    x = (border_point1[0] + border_point2[0])/2
                    if w >= 2.0 and h >= 2.0:
                        self.current_holes.append([x, border_point1[1], border_point1[2], w, h])
                        print([x, border_point1[1], border_point1[2], w, h])
                k += 1

    def get_error(self, p11, p12, p2):
        p11 = np.array(p11)
        p12 = np.array(p12)
        p2 = np.array(p2)
        print('point 1', p11, [p2[0], p2[1], p2[2]])
        print('point 2', p12, [p2[0], p2[1]+p2[3], p2[2]+p2[3]])
        er_p1 = p11 - [p2[0], p2[1], p2[2]]
        er_p2 = p12 - [p2[0], p2[1]+p2[3], p2[2]+p2[3]]
        return er_p1, er_p2

    def get_poses_in_holes(self):
        self.poses_in_hole = []
        self.prepose_to_hole = []

        # check avaliable space for drone in hole
        print('number holes', len(self.current_holes))
        for i in range(len(self.current_holes)):
            n_drone_y = int(self.current_holes[i][3] / self.min_size_drone[0])
            n_drone_z = int(self.current_holes[i][4] / self.min_size_drone[1])
            print('n_y =', n_drone_y, 'n_z =', n_drone_z,
                  'n =', n_drone_y * n_drone_z)

            y0_hole = self.current_holes[i][1]
            z0_hole = self.current_holes[i][2]
            print('left up coord of hole', y0_hole, z0_hole)

            # calculate dist between drones for current hole
            len_rec_y = self.current_holes[i][3] / n_drone_y
            len_rec_z = self.current_holes[i][4] / n_drone_z
            print("n_drone in y and z ", len_rec_y, len_rec_z)

            # get postions in holes
            poses_in_hole_current = []
            pre_glob_poses_in_hole = []
            for y in range(n_drone_y):
                for z in range(n_drone_z):
                    y_hp = y0_hole - (y*len_rec_y) - (len_rec_y/2)
                    z_hp = z0_hole - (z*len_rec_z) - (len_rec_z/2)
                    poses_in_hole_current.append(
                        [self.current_holes[i][0] - 1, y_hp, z_hp])
                    pre_glob_poses_in_hole.append(
                        [self.current_holes[i][0] - 2, y_hp, z_hp])

            self.poses_in_hole.append(poses_in_hole_current)
            self.prepose_to_hole.append(pre_glob_poses_in_hole)

        for i in range(len(self.poses_in_hole)):
            print(i, 'hole pose', self.poses_in_hole[i])
