import cv2
import time
import rospy
import airsim
import numpy as np
import math
from threading import Thread
from scipy.spatial.transform import Rotation as R

from utils.controller import dronePositionController
from utils.helpers import *
from utils.cv import *


class Action():
    def __init__(self) -> None:
        self.type = 0
        self.data = None

    def __call__(self):
        return self.data


class ActionWall(Action):
    def __init__(self, holes: list, yaw: float) -> None:
        super().__init__()
        self.type = 2
        self.data = [holes, yaw]


class ActionCenter(Action):
    def __init__(self, position: np.ndarray, yaw: float) -> None:
        super().__init__()
        self.type = 1
        self.data = [position, yaw]


class HeadingDroneController:
    def __init__(self, pid_params) -> None:
        rospy.on_shutdown(self.on_shutdown_cb)
        self.airsim_client = airsim.MultirotorClient()
        self.airsim_client.confirmConnection()
        self.run_cv = True

        self.yaw_correct = 120

        self.actions = []
        # ---for holes detection---
        self.rad = math.pi / 180
        self.koef = 1.5
        self.current_holes = []
        self.poses_in_hole = []
        self.is_hole_detected = False
        self.min_size_drone = [1.5, 1.5]  # Y, Z metes for one drone
        self.detect_next_turn = False
        self.orientation = 0
        self.orient_buf = 0
        self.current_point_id = 0
        self.hole_in_front = False
        self.elevator = 0  # 1 up, -1 down, 0 same

        self.cameraToDrone = np.array([
            [0.0000328, 0.0000000, 1.0000000],
            [-1.0000000, 0.0000328, 0.0000328],
            [-0.0000328, -1.0000000, 0.0000000],
        ])
        self.fmlRot = np.array([
            [0.8660254,  0.0000000,  0.5000000],
            [0.0000000,  1.0000000,  0.0000000],
            [-0.5000000,  0.0000000,  0.8660254],
        ])
        # -------

        self.run_cv_task = False
        self.path_found = False
        self.previous_pose = None
        self.freq = 30
        self.name = self.airsim_client.listVehicles()[0]
        self.id = 1
        self.controller = dronePositionController(
            self.id, pid_params, pub_rate=self.freq)
        self.waypoints = [np.array([10, 0, 10])]

        self.task = Thread(target=self.loop)
        self.task.start()

    def on_shutdown_cb(self):
        self.run_cv = False
        time.sleep(0.5)
        del self
        rospy.logfatal("shutdown")

    def __del__(self):
        self.task.join()

    def loop(self) -> None:
        rospy.loginfo("Leader drone started")
        while(self.run_cv):
            if not self.controller.is_armed:
                self.controller.set_arm(True)
                while not self.controller.takeoff():
                    continue

            if not self.run_cv_task and self.controller.is_arrived:
                self.target = self.controller.get_global_position() + \
                    np.array([10., 0., 0.])
                print(self.target)
                self.controller.set_yaw(self.yaw_correct)
                self.controller.set_position(self.target)
                time.sleep(5.0)

                self.previous_pose = self.controller.get_global_position()
                self.run_cv_task = True
                continue

            if self.run_cv_task:
                depth, delta, _ = self.aquire_depth_and_distance_to_wall()
                self.checking_turns(depth, delta)
                # print('hole_in_front', self.hole_in_front)

                if self.hole_in_front:
                    len_detection = 14.0

                    if delta > len_detection:
                        point = self.moving_small_steps(
                            self.controller.get_global_position())  # get next small point
                        self.controller.set_position_yaw(
                            point, self.yaw_correct + self.orientation)
                    else:
                        time.sleep(2.0)
                        depth, _, img_pc = self.aquire_depth_and_distance_to_wall()
                        self.depth_hole_detection(
                            depth, self.get_transform(), img_pc, th=20)

                        if len(self.current_holes):
                            self.get_poses_in_holes()
                            # move in front of the hole
                            point_in_front_wall = self.controller.get_global_position(
                            )
                            next_point = self.prepose_to_hole[0][0]
                            self.controller.set_position(next_point)
                            while self.achieved_point(next_point, self.controller.get_global_position(), 0.2):
                                time.sleep(0.1)

                            # move_through the hole
                            move_through = 4.0  # meters
                            next_point = self.prepose_to_hole[0][0] + \
                                np.array(self.get_pose_shift(move_through))
                            self.controller.set_position_yaw(
                                next_point, self.yaw_correct + self.orientation)
                            while self.achieved_point(next_point, self.controller.get_global_position(), 0.1):
                                time.sleep(0.1)

                            # move to center before the wall
                            l_before_wall = 16.0
                            next_point = point_in_front_wall + \
                                np.array(self.get_pose_shift(l_before_wall))
                            self.controller.set_position_yaw(
                                next_point, self.yaw_correct + self.orientation)
                            while self.achieved_point(next_point, self.controller.get_global_position(), 0.1):
                                time.sleep(0.1)
                        else:
                            rospy.logwarn('Not holes')

                        self.clear_var()

                if not self.hole_in_front:
                    len_detection = 10.0

                    if self.elevator == 0:
                        if delta > len_detection:
                            point = self.moving_small_steps(
                                self.controller.get_global_position())  # get next small point
                            self.controller.set_position_yaw(
                                point, self.yaw_correct + self.orientation)
                        else:
                            self.orientation += self.orient_buf

                            # print('rotating', self.orientation)
                            self.controller.set_position_yaw(
                                self.controller.get_global_position(), self.yaw_correct + self.orientation)
                            rospy.loginfo("End tunnel, next one")

                            time.sleep(3.0)

                            # --------ADD WAY POINT and ORIINTATION --------
                            self.actions.append(ActionCenter(
                                self.controller.get_global_position(), self.orientation))
                            # ----------------------------------------------

                            len_detection = 14.0
                            _, delta, _ = self.aquire_depth_and_distance_to_wall()
                            if delta <= len_detection:
                                delta_wall = delta - len_detection
                                next_point = self.controller.get_global_position(
                                ) + np.array(self.get_pose_shift(delta_wall))
                                self.controller.set_position(next_point)
                                while self.achieved_point(next_point, self.controller.get_global_position(), 0.1):
                                    time.sleep(0.1)
                            self.clear_var()

                    if self.elevator == 1:
                        next_point = self.moving_small_steps(
                            self.controller.get_global_position(), dir=0, z=10)  # get next small point
                        self.controller.set_position_yaw(
                            point, self.yaw_correct + self.orientation)
                        while self.achieved_point(next_point, self.controller.get_global_position(), 0.1):
                            time.sleep(0.1)
                        self.elevator == 0

            time.sleep(1./self.freq)

    def clear_var(self):
        self.current_holes = []
        self.poses_in_hole = []
        self.is_hole_detected = False
        self.hole_in_front = False
        self.detect_next_turn = False
        self.current_point_id += 1

    def get_pose_shift(self, k):
        return [k * math.cos(self.orientation * self.rad), k * math.sin(self.orientation * self.rad), 0]

    def get_transform(self):
        dronePos = np.array(
            [self.controller.get_global_position()])
        droneRot = R.from_quat(
            self.controller.get_global_orientation()).as_matrix()
        droneRot = np.dot(droneRot, self.cameraToDrone)
        droneRot = np.dot(droneRot, self.fmlRot)
        droneToWorld = np.concatenate((droneRot, dronePos.T), axis=1)
        droneToWorld = np.concatenate(
            (droneToWorld, np.array([[0., 0., 0., 1.]])))
        return droneToWorld
        # end

    def aquire_depth_and_distance_to_wall(self):
        responses = self.airsim_client.simGetImages([
            airsim.ImageRequest('0', airsim.ImageType.Scene, compress=True),
            airsim.ImageRequest(
                '1', airsim.ImageType.DepthPerspective, True, compress=False),
            airsim.ImageRequest('2', airsim.ImageType.Segmentation, False, compress=False)],
            vehicle_name=self.name)

        semantic = np.fromstring(responses[2].image_data_uint8, dtype=np.uint8)
        semantic = np.reshape(
            semantic, (responses[2].height, responses[2].width, 3))

        # print(np.unique(semantic[0]), np.unique(semantic[1]), np.unique(semantic[2]))
        #_, th = cv2.threshold(semantic, 200, 255, cv2.THRESH_BINARY)
        colors = []
        # print('semantic', np.unique(semantic[:, :, 0]),  np.unique(semantic[:, :, 1]),  np.unique(semantic[:, :, 2]))
        if np.any(semantic[20:124, 100:155] >= 200):
            self.hole_in_front = True
            self.detect_next_turn = True
            self.orient_buf = 0
        else:
            self.hole_in_front = False
            self.detect_next_turn = False

        img1d = np.array(responses[1].image_data_float, dtype=np.float)
        img1d[img1d > 255] = 255
        img2d = np.reshape(
            img1d, (responses[1].height, responses[1].width))
        img_pc, _ = depth_to_pc(img2d)
        droneToWorld = self.get_transform()
        wall_roi = [20, responses[1].height - 60, 100,
                    responses[1].width - 100]  # x1, x2, y1, y2
        wall_pc = img_pc[wall_roi[0]:wall_roi[1],
                         wall_roi[2]:wall_roi[3]]
        wall_pc = np.reshape(
            wall_pc, (wall_pc.shape[0]*wall_pc.shape[1], wall_pc.shape[2]))
        wall_pc = pc_transform(wall_pc, droneToWorld)

        delta = np.abs(np.min(wall_pc[:, abs(int(math.sin(self.orientation*self.rad)))]) -
                       self.controller.get_global_position()[abs(int(math.sin(self.orientation*self.rad)))])
        rospy.logwarn("%f", delta)

        # get depth image
        depth = img1d.reshape(responses[1].height, responses[1].width)
        depth = np.array(depth, dtype=np.uint8)

        return depth, delta, img_pc

    def achieved_point(self, tp, cp, tol):
        achieved = False
        # print(np.linalg.norm(tp - cp))
        if np.linalg.norm(tp - cp) > tol:
            achieved = True
        return achieved

    def moving_small_steps(self, dronePos, dir=1, z=0):
        point = dronePos + np.array(
            [dir * math.cos(self.orientation * self.rad) * self.koef, dir * math.sin(self.orientation * self.rad) * self.koef, z])

        # keep y coordinate while moving x
        if abs(int(math.sin(self.orientation*self.rad))) == 0:
            point[1] = self.previous_pose[1]
        # keep x coordinate while moving y
        if abs(int(math.sin(self.orientation*self.rad))) == 1:
            point[0] = self.previous_pose[0]
        # keep z #TODO
        point[2] = self.previous_pose[2]
        self.previous_pose = point
        return point

    def checking_turns(self, depth, delta):
        th_l = 19
        th_r = 19
        th_d = 30

        if delta >= 14:
            wall_l = depth[0:144, 0:45]  # left
            wall_r = depth[0:144, 210:255]  # right
            wall_d = depth[130:143, 100:150]  # down
            wall_f = depth[20:124, 100:155]  # forward

            if np.max(wall_l) > th_l and not self.detect_next_turn:
                self.detect_next_turn = True
                self.orient_buf = 90
                print('next left')
            if np.max(wall_r) > th_r and not self.detect_next_turn:
                self.detect_next_turn = True
                self.orient_buf = -90
                print('next right')
            if np.max(wall_d) > th_d and not self.detect_next_turn:
                self.detect_next_turn = True
                self.orient_buf = 0
                print('next down')

            # if np.max(wall_l) <= th_l and np.max(wall_l) <= th_r and np.max(wall_d) <= th_d and not self.hole_in_front and not self.detect_next_turn:
            #     self.elevator = 1

            # if np.max(wall_l) <= th_l and np.max(wall_l) <= th_r and np.max(wall_d) >= th_d and not self.hole_in_front and not self.detect_next_turn:
            #     self.elevator = -1

            # print('average L', np.mean(wall_l), "R", np.mean(
            #     wall_r), "D", np.mean(wall_d), 'F', np.max(wall_f), 'or_b', self.orient_buf, 'or', self.orientation, 'el', self.elevator)

    def depth_hole_detection(self, depth, droneToWorld, img_pc, th=20):
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
                    #print('Contour №', k)
                    # rect = cv2.minAreaRect(contour)
                    # box = cv2.boxPoints(rect)
                    # box = np.int0(box)

                    box = cv2.boundingRect(contour)

                    #print(box[0], box[1], box[2], box[3])

                    # fix it, if box[0] = 144 of box[1] = 256, index are not in range
                    if box[0] == 144:
                        box[0] -= 1
                    if box[1] == 256:
                        box[1] -= 1
                    l = 0
                    if box[1] + box[3] == 144:
                        l = -1
                    if box[0] + box[2] == 256:
                        l = -1

                    border_point1 = img_pc[box[1], box[0]]
                    border_point2 = img_pc[box[1] +
                                           box[3] + l, box[0] + box[2] + l]
                    border_points = pc_transform(
                        [border_point1, border_point2], droneToWorld)
                    border_point1 = border_points[0]
                    border_point2 = border_points[1]
                    #print(border_point1, border_point2)

                    # current_holes: x_wall, y_left_up, z_left_up, w, h,
                    if self.orientation == 0:
                        w = abs(border_point2[1] - border_point1[1])
                        x = (border_point1[0] + border_point2[0])/2
                        y = border_point1[1]

                    if self.orientation == 90:
                        w = abs(border_point2[0] - border_point1[0])
                        x = border_point1[0]
                        y = (border_point1[1] + border_point2[1])/2

                    z = border_point1[2]
                    h = abs(border_point2[2] - border_point1[2])

                    if w >= self.min_size_drone[0] and h >= self.min_size_drone[1]:
                        self.current_holes.append(
                            [x, y, z, w, h])
                        print([x, y, z, w, h])
                k += 1
        if not self.is_hole_detected:
            print('Nothing.')

    def get_error(self, p11, p12, p2):
        p11 = np.array(p11)
        p12 = np.array(p12)
        p2 = np.array(p2)
        #print('point 1', p11, [p2[0], p2[1], p2[2]])
        #print('point 2', p12, [p2[0], p2[1]+p2[3], p2[2]+p2[3]])
        er_p1 = p11 - [p2[0], p2[1], p2[2]]
        er_p2 = p12 - [p2[0], p2[1]+p2[3], p2[2]+p2[3]]
        return er_p1, er_p2

    def get_poses_in_holes(self):
        self.poses_in_hole = []
        self.prepose_to_hole = []

        # check avaliable space for drone in hole
        #print('number holes', len(self.current_holes))
        for i in range(len(self.current_holes)):
            n_drone_y = int(self.current_holes[i][3] / self.min_size_drone[0])
            n_drone_z = int(self.current_holes[i][4] / self.min_size_drone[1])
            # print('n_y =', n_drone_y, 'n_z =', n_drone_z,
            #       'n =', n_drone_y * n_drone_z)

            y0_hole = self.current_holes[i][1]
            z0_hole = self.current_holes[i][2]
            # print('left up coord of hole', y0_hole, z0_hole)

            # calculate dist between drones for current hole
            len_rec_y = self.current_holes[i][3] / n_drone_y
            len_rec_z = self.current_holes[i][4] / n_drone_z
            # print("n_drone in y and z ", len_rec_y, len_rec_z)

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

        key_points_in_holes = [
            np.array(self.poses_in_hole[i][0]) for i in range(len(self.poses_in_hole))]
        self.actions(ActionWall(key_points_in_holes, self.orientation))
