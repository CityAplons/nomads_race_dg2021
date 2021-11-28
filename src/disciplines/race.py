import sys
import cv2
import time
import rospy
import airsim
import numpy as np
import math
from threading import Thread
from scipy.spatial.transform import Rotation as R

from utils.controller import droneVelocityController
from utils.helpers import *
from utils.cv import *

sys.setrecursionlimit(1500)
freq = 20
yaw_corret = 30
way_hole_poses = np.array([[48, 0.5, 9], [88, -1.5, 10]])


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

        self.run_cv = True
        self.target = np.array([10., 0., 3.])

        self.previous_pose = None

        self.run_cv_task = False
        self.path_found = False

        # ---for holes detection---
        self.current_holes = []
        self.poses_in_hole = []
        self.is_hole_detected = False
        self.min_size_drone = [1.5, 1.5]  # Y, Z metes for one drone
        self.check_angle = [-90, 90]
        self.check_id = 0
        self.detect_next_turn = False
        self.orientation = 0
        self.orient_buf = 0
        self.current_point_id = 0
        # -------

        self.instances = []
        for id, vehicle in enumerate(self.airsim_client.listVehicles(), 1):
            rospy.loginfo("Airsim instance %d: %s", num, vehicle)
            first_point = np.array([20, 0, (12. - 2*id)])
            instance = {
                "name": vehicle,
                "id": id,
                "controller": droneVelocityController(id, pid_params, max_vel=5, pub_rate=freq),
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
                    instance["controller"].set_target_position(self.target)
                    time.sleep(6000.0)
                elif not self.run_cv_task:
                    instance["controller"].set_target_yaw((yaw_corret + self.orientation) * 0.0174532925)
                    print('get global pose',
                          instance["controller"].get_global_position())
                    self.previous_pose = instance["controller"].get_global_position(
                    )
                    instance["controller"].set_target_yaw((yaw_corret + self.orientation) * 0.0174532925)
                    instance["controller"].set_target_position(instance["controller"].get_global_position())
                    self.cv_request_path(
                        instance["controller"].get_global_position() + np.array([30., 0., 0.]))
            # wait for stabilasing
            
            # Re-calculate center pose location
            formation_center_pose += instance["controller"].get_global_position() / \
                self.num

        self.formation_center_pose = formation_center_pose

    def cv_request_path(self, point):
        self.target = point
        self.run_cv_task = True

    def cv_thread(self, instance: dict):
        rad = math.pi / 180

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
                    [-1.0000000, 0.0000328, 0.0000328],
                    [-0.0000328, -1.0000000, 0.0000000],
                ])
                fmlRot = np.array([
                    [0.8660254,  0.0000000,  0.5000000],
                    [0.0000000,  1.0000000,  0.0000000],
                    [-0.5000000,  0.0000000,  0.8660254],
                ])

                dronePos = np.array(
                    [instance["controller"].get_global_position()])
                droneRot = R.from_quat(
                    instance["controller"].get_global_orientation()).as_matrix()
                droneRot = np.dot(droneRot, cameraToDrone)
                droneRot = np.dot(droneRot, fmlRot)
                droneToWorld = np.concatenate((droneRot, dronePos.T), axis=1)
                droneToWorld = np.concatenate(
                    (droneToWorld, np.array([[0., 0., 0., 1.]])))

                wall_roi = [20, responses[1].height - 20, 100,
                            responses[1].width - 100]  # x1, x2, y1, y2
                wall_pc = img_pc[wall_roi[0]:wall_roi[1],
                                 wall_roi[2]:wall_roi[3]]
                wall_pc = np.reshape(
                    wall_pc, (wall_pc.shape[0]*wall_pc.shape[1], wall_pc.shape[2]))
                wall_pc = pc_transform(wall_pc, droneToWorld)

                delta = np.abs(np.min(wall_pc[:, int(math.sin(self.orientation*rad))]) - dronePos[0, int(math.sin(self.orientation*rad))])

                # moving to wall
                if delta > 10.0:
                    rospy.logwarn("%f", delta)
                    koef = 0.9*(delta - 10.0)
                    point = dronePos[0] + np.array(
                        [math.cos(self.orientation * rad) * koef, math.sin(self.orientation * rad) * koef, 0])

                    # keep y coordinate while moving x
                    if int(math.sin(self.orientation*rad)) == 0:
                        point[1] = self.previous_pose[1]
                    # keep x coordinate while moving y
                    if int(math.sin(self.orientation*rad)) == 1:
                        point[0] = self.previous_pose[0]
                    # keep z
                    point[2] = self.previous_pose[2]
                    self.previous_pose = point
                    instance["controller"].set_target_yaw((yaw_corret + self.orientation) * 0.0174532925)
                    instance["controller"].set_target_position(point)

                    # check for turns in labyrinth
                    depth = img1d.reshape(
                        responses[1].height, responses[1].width)
                    depth = np.array(depth, dtype=np.uint8)
                    self.checking_turns(depth, delta)

                if delta <= 10.0:
                    # delay to stabilaze drone
                    time.sleep(3.0)

                    responses = self.airsim_client.simGetImages([
                        airsim.ImageRequest(
                            '0', airsim.ImageType.Scene, compress=False),
                        airsim.ImageRequest('1', airsim.ImageType.DepthPerspective, True, False)], vehicle_name=instance["name"])
                    img1d = np.array(responses[1].image_data_float, dtype=np.float)
                    img1d[img1d > 255] = 255
                    img2d = np.reshape(
                        img1d, (responses[1].height, responses[1].width))
                    img_pc, drone_pc = depth_to_pc(img2d)

                    dronePos = np.array(
                        [instance["controller"].get_global_position()])
                    droneRot = R.from_quat(
                        instance["controller"].get_global_orientation()).as_matrix()
                    droneRot = np.dot(droneRot, cameraToDrone)
                    droneRot = np.dot(droneRot, fmlRot)
                    droneToWorld = np.concatenate((droneRot, dronePos.T), axis=1)
                    droneToWorld = np.concatenate(
                        (droneToWorld, np.array([[0., 0., 0., 1.]])))

                    # --- holes detection and poses calculation ---
                    depth = img1d.reshape(
                        responses[1].height, responses[1].width)
                    depth = np.array(depth, dtype=np.uint8)

                    self.depth_hole_detection(
                        depth, droneToWorld, img_pc, th=20)

                    if self.is_hole_detected:  # works
                        self.get_poses_in_holes()
                        print(self.poses_in_hole[0][0])

                        # TODO
                        print('go to ', self.poses_in_hole[0][0])

                    if not self.is_hole_detected:  # works
                        self.orientation = self.orient_buf
                        print('rotating', self.orientation)
                        instance["controller"].set_target_yaw((yaw_corret + self.orientation) * 0.0174532925)
                        instance["controller"].set_target_position(instance["controller"].get_global_position())

                if self.is_hole_detected:
                    instance["controller"].set_target_position(
                        self.poses_in_hole[0][0])
                    # Waiting for arrival to point
                    while self.achieved_point(self.poses_in_hole[0][0], instance["controller"].get_global_position(), 0.1):
                        pass

                    move_through = 5.0  # meters
                    next_point = self.poses_in_hole[0][0] + np.array([move_through * math.cos(
                        self.orientation * rad), move_through * math.sin(self.orientation * rad), 0])

                    print('Go through hole, next point', next_point)
                    instance["controller"].set_target_yaw((yaw_corret + self.orientation) * 0.0174532925)
                    instance["controller"].set_target_position(next_point)
                    # Waiting for arrival to point
                    while self.achieved_point(next_point, instance["controller"].get_global_position(), 0.1):
                        pass
                    rospy.logwarn("end wall, next one")

                    self.current_holes = []
                    self.poses_in_hole = []
                    self.run_cv = True
                    self.check_id = 0
                    self.is_hole_detected = False
                    self.current_point_id += 1
            else:
                time.sleep(0.5)
            time.sleep(0.03)

    def achieved_point(self, tp, cp, tol):
        achieved = False
        # print(np.linalg.norm(tp - cp))
        if np.linalg.norm(tp - cp) > tol:
            achieved = True
        return achieved

    def checking_turns(self, depth, delta):
        if delta >= 20:
            wall_l = depth[40:100, 0:30]  # left
            wall_r = depth[40:100, 220:255]  # right
            wall_d = depth[130:143, 100:150]  # down
            wall_f = depth[40:100, 50:200]  # forward
            # print('average L', np.mean(wall_l), "R", np.mean(wall_r), "D",np.mean(wall_d), 'W', np.max(wall_w))
            if np.mean(wall_l) > 35 and not self.detect_next_turn:
                self.detect_next_turn = True
                self.orient_buf += 90
                print('next left')
            if np.mean(wall_r) > 35 and not self.detect_next_turn:
                self.detect_next_turn = True
                self.orient_buf += -90
                print('next right')
            if np.max(wall_d) > 40 and not self.detect_next_turn:
                print('next down')
                self.detect_next_turn = True
                self.orient_buf += 0

    def depth_hole_detection(self, depth, droneToWorld, img_pc, th=20):
        rad = math.pi / 180
        (T, thresh) = cv2.threshold(depth, th, 255, cv2.THRESH_BINARY_INV)
        contours, hierarchy = cv2.findContours(
            thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        # check how mush counters we detected. Are they there or not?
        if len(contours) > 1:
            self.is_hole_detected = True
        if len(contours) <= 1:
            self.is_hole_detected = False
        print('find counters n =', len(contours) - 1, self.is_hole_detected)

        # if YES
        if self.is_hole_detected:
            k = 0  # counter
            for contour in contours:
                if k > 0:
                    print('Contour №', k)
                    rect = cv2.minAreaRect(contour)
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                    print(box[0], box[1], box[2], box[3])

                    # get world position of from pixels
                    print(img_pc.shape)
                    border_points = [ img_pc[p[1], p[0]] for p in box ]
                    border_points = pc_transform(
                        border_points, droneToWorld)
                    print("border_points:", border_points)
                    
                    # current_holes: x_wall, y_left_up, z_left_up, w, h,
                    if self.orientation == 0:
                        w = abs(border_points[1][1] - border_points[0][1])
                        x = (border_points[0][0] + border_points[2][0])/2
                        y = border_points[0][1]

                    if self.orientation == 90:
                        w = abs(border_points[1][0] - border_points[0][0])
                        x = border_points[0][0]
                        y = (border_points[0][1] + border_points[2][1])/2

                    z = border_points[0][2]
                    h = abs(border_points[2][2] - border_points[0][2])
                    
                    if w >= self.min_size_drone[0] and h >= self.min_size_drone[1]:
                        self.current_holes.append(
                            [x, y, z, w, h])
                        print([x, y, z, w, h])
                k += 1

        if not self.is_hole_detected:
            print('Nothing.', self.orientation)

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
