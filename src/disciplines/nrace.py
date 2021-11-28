import time
import rospy
import numpy as np
import math
import copy
from threading import Thread
from scipy.spatial.transform import Rotation as R

from utils.controller import dronePositionController
from utils.leader import HeadingDroneController, ActionWall, ActionCenter
from utils.helpers import *

freq = 30
yaw_corret = 120

def nrace_init(count: int, pid_params: dict = None):
    rospy.loginfo("Nikita's RACE node started")
    formation = RaceFormation(count, pid_params)
    rate = rospy.Rate(freq)
    while not rospy.is_shutdown():
        formation.loop()
        rate.sleep()

class RaceFormation:

    def __init__(self, num, pid_params: dict = None) -> None:
        self.num = num
        self.target = np.array([0., 0., 3.])
        
        self.uavHeight = self.uavWidth = self.heightSafeGap = self.widthSafeGap = 1.

        self.uavRows=int((self.num-1)**0.5)
        self.uavColumns=math.ceil((self.num-1)/self.uavRows)
        self.heightDistance=(self.uavHeight+self.heightSafeGap)*(self.uavRows-1)
        self.widthDistance=(self.uavWidth+self.widthSafeGap)*(self.uavColumns-1)

        self.boss = HeadingDroneController(pid_params)

        self.instances = []
        if self.num > 1:
            for id in range(2, self.num):
                instance = {
                    "id": id,
                    "controller": dronePositionController(id, pid_params, pub_rate=freq),
                    "waypoints": []
                }
                self.instances.append(instance)

        # Initial formation
        initial_shifts = self.initialFormation((self.num-1), np.array([0,0,0]), 0, 0)
        for i, instance in enumerate(self.instances):
                instance["waypoints"].append((np.array([0,0,10]) + initial_shifts[i]))

    def loop(self) -> None:

        self.arrived_num = 0
        formation_center_pose = np.array([0, 0, 0], dtype=float)

        if len(self.boss.actions):
            current_action = self.boss.actions.pop(0)
            
            f_positions = None
            if current_action is ActionCenter:
                data = current_action()
                position = data[0]
                yaw = data[1]
                f_positions = self.initialFormation((self.num-1), position, yaw, 0)
            elif current_action is ActionWall:
                data = current_action()
                wall_points = data[0]
                yaw = data[1]
                f_positions = self.flyingThroughHole(wall_points, (self.num-1), yaw)

            for i, instance in enumerate(self.instances):
                instance["waypoints"].append(f_positions[i])

                # Takeoff
                if not instance["controller"].is_armed:
                    instance["controller"].set_arm(True)
                    while not instance["controller"].takeoff():
                        rospy.sleep(0.1)
                    while not instance["controller"].is_arrived:
                        rospy.sleep(0.1)

                self.arrived_num += 1

                if len(instance["waypoints"]):
                    rospy.loginfo("Following waypoint")
                    target = instance["waypoints"].pop(0)
                    instance["controller"].set_yaw(yaw_corret)
                    instance["controller"].set_position(target)
                    time.sleep(15.0)
                    

                # Re-calculate center pose location
                formation_center_pose += instance["controller"].get_global_position() / \
                    self.num

            self.formation_center_pose = formation_center_pose
    
    def initialFormation(self, robotAmount, firstRobotCenterPosition, yaw, distanceToLeader):
        formationPositions=np.array([firstRobotCenterPosition])
        # print(firstRobotCenterPosition,' ',heightDistance,' ',widthDistance)
        robotHeading = 1 if yaw==0 and yaw==-180 else 0

        for i in range(robotAmount):
            robotCoordinates=np.array([firstRobotCenterPosition])
            robotCoordinates[0,abs(int(robotHeading)-1)]-=distanceToLeader
            if self.uavColumns>1:
                robotCoordinates[0,robotHeading]+=self.widthDistance/2-(self.uavWidth+self.widthSafeGap)*float(i-self.uavColumns*(i//(self.uavColumns)))
            if self.uavRows>1: robotCoordinates[0,2]+=(self.heightDistance/2-(self.uavHeight+self.heightSafeGap)*float(i//self.uavColumns)) # ((-1)**(i//6))*
            formationPositions=np.vstack([formationPositions,robotCoordinates])
        return formationPositions # formationPositions=[[x,y,z],[x,y,z],...] - formation positions

    def flyingThroughHole(self, accessiblePoints, robotAmount, yaw):
        # multiplyer to count axes direction (+ or -)
        robotHeading = 1 if yaw==0 and yaw==-180 else 0
        znak = 1 if yaw==0 and yaw==-90 else -1
        robotHeading=abs(int(robotHeading)-1)
        for i in range(robotAmount):
            robotCoordinates = copy.deepcopy(accessiblePoints[int(i-(accessiblePoints.shape[0])*(i // (accessiblePoints.shape[0]))),:])
            robotCoordinates[robotHeading] -= znak*1*float(i // accessiblePoints.shape[0]) # 1 meter distance beetwen uav groups
            robotPosition=copy.deepcopy(robotCoordinates) if i == 0 else np.vstack([robotPosition,robotCoordinates])
        return robotPosition
