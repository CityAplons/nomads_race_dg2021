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


def nrace_init(count: int, pid_params: dict = None):
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

        self.run_cv_task = False
        self.path_found = False

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
                    self.cv_request_path(
                        instance["controller"].get_global_position() + np.array([20., 0., 0.]))
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

                # wall_roi = [30, responses[1].height - 30, 80, responses[1].width - 80 ]
                # wall_pc = img_pc[wall_roi[0]:wall_roi[1], wall_roi[2]:wall_roi[3]]
                # wall_pc= np.reshape(wall_pc, (wall_pc.shape[0]*wall_pc.shape[1], wall_pc.shape[2]))
                # wall_pc = pc_transform(wall_pc, droneToWorld)
                # delta = np.abs(np.min(wall_pc[:, 0]) - dronePos[0,0])
                # if delta > 10.0:
                #     rospy.logwarn("%f", delta)
                #     delta -= 8.1
                #     point = dronePos[0] + np.array([0.3*delta, 0, 0])
                #     instance["controller"].set_position(point)
                #     self.run_cv_task = False

                world_pc = pc_transform(drone_pc, droneToWorld)
                roi = world_pc - dronePos[0]
                trimed_ids = np.argwhere(
                    (roi[:, 2] > 1)&
                    (roi[:, 2] < 18)&
                    (roi[:, 0] < 40)&
                    (roi[:, 1] < 40))
                max_pos = np.amax(roi[trimed_ids], axis=0)[0]
                #target = self.target
                target = max_pos
                self.target = target
                target_yaw = np.arctan2(target[1], target[0])
                print(target, target_yaw)

                instance["controller"].set_yaw(target_yaw*2*np.pi)
                instance["controller"].set_position(instance["controller"].get_global_position())
                time.sleep(3)

                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(world_pc)
                 # o3d.io.write_point_cloud("/home/nikita/pc" + str(rospy.get_time()) + ".xyz", pcd)
                voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(
                    pcd,
                    voxel_size=0.6
                )
                # o3d.visualization.draw_geometries([voxel_grid])
                #path_planner = AStar(voxel_grid)

                cur_position = instance["controller"].get_global_position()
                shifted = np.array([cur_position[0], cur_position[1], 0])
                search_volume = Area(shift=shifted, min_y=-40, max_y=40)
                if search_volume.within_area(target):
                    path_planner = RRTStar(
                        dronePos[0], voxel_grid, 7, search_volume, resolution=20, it_limit=1500)
                    path = path_planner.search(target)
                    if path is not None:
                        #path = path_planner.smooth_path(path)
                        instance["waypoints"] = path[1:]
                        self.path_found = True
                        self.run_cv_task = False
                else:
                    rospy.logwarn("Target position out of space")
                    time.sleep(0.5)
            else:
                time.sleep(0.5)
            time.sleep(0.03)
