import os
import math
import time
import rospy
import airsim
import numpy as np
import cv2

from utils.g_uav_controller import UAVController
# from utils.controller import dronePositionController

import time

freq = 30

def grace_init(count:int, pid_params: dict = None):
    rospy.loginfo("RACE node started")
    formation = RaceFormation(count, pid_params)
    rate = rospy.Rate(freq)
    while not rospy.is_shutdown():
        formation.loop()
        rate.sleep()
    for inst in formation.instances:
        while not inst["controller"].land():
            continue
    # for inst in formation.instances:
    #     while inst["controller"].current_state != "Grounded":
    #         print(inst["controller"].current_state)
        inst["controller"].arming_client(False)

class RaceFormation:
    ## TODO:
    #   - Assign unified formation for swarm
    #   - Implement state-machine handlers: takeoff, outlook, move
    #   - Implement direction finfing algorithm (opencv)

    def __init__(self, num, pid_params: dict = None) -> None:
        self.num = num
        self.airsim_client = airsim.MultirotorClient()
        self.airsim_client.confirmConnection()
        self.arrived_num = 0
        self.follow_azimuth = 0
        self.formation_center_pose = np.array([0, 0, 0])

        self.state_machine = {
            "disarm": self.arm,
            "armed": self.nop,
            "takeoff": self.takeoff,
            "tookoff": self.outlook,
            "arrival": self.move,
            "finished": self.land,
            "break": self.end
        }
        self.once = False

        self.instances = []
        for num, vehicle in enumerate(self.airsim_client.listVehicles(), 1):
            rospy.loginfo("Airsim instance %d: %s", num, vehicle)
            instance = {
                "name": vehicle,
                "id": num,
                # "controller": dronePositionController(id, pid_params, pub_rate=freq)
                "controller": UAVController(num, pid_params),
            }
            first_point = np.array([10., 0., (18 - 2.*num)])
            self.instances.append(instance)
            print('cho '+str(num))

        #new
        self.current_pose=None
        self.qty=1
        self.distance=1.0
        self.state=0

        airsim.wait_key('Press any key to execute RACE task')
    
    # Requesting a task from a user
    # def user_input(self):
    #     try:
    #         x1, y1, z1 = raw_input("Enter x, y, z: ").split(' ')
    #         self.current_pose=[float(x1),float(y1),float(z1)] 
    #     except SyntaxError or ValueError:
    #         #pass
    #         self.state=2
    #         self.current_pose=[-2,0,3]

    def loop(self) -> None:

        self.arrived_num = 0
        formation_center_pose = np.array([0, 0, 0], dtype=float)

        for instance in self.instances:

            instance["controller"].set_mode("OFFBOARD")
            print(instance["controller"].state)
            if (instance["controller"].current_status == "Init") or (instance["controller"].current_status == "None") : instance["controller"].arm()
            print(instance["controller"].state)
            if (instance["controller"].current_status != "Hover") : instance["controller"].takeoff()
            if (instance["controller"].current_status == "Hover") : instance["controller"].hover(0.6)

            # # Takeoff
            # if not instance["controller"].is_armed:
            #     instance["controller"].set_arm(True)
            #     while not instance["controller"].takeoff():
            #         continue

            # State machine execution
            current_state = instance["controller"].current_state
            rospy.loginfo("[%s] Current state: %s", instance["name"], current_state)
            
            # if current_state in self.state_machine:
            #     self.state_machine[current_state](instance["controller"])
            # else:
            #     rospy.logerr("Undefined state: %s", instance["controller"].state)

            # Set point for autopilot
            
            # Re-calculate center pose location
            formation_center_pose += instance["controller"].pose / self.num

        self.formation_center_pose = formation_center_pose

    def arm(self, controller: UAVController):
        controller.arm()

    def takeoff(self, controller: UAVController):
        # controller.current_waypoint = controller.start_waypoint + np.array([3., 0., 0.])
        controller.takeoff()


    def outlook(self, controller: UAVController):
        # controller.set_pos(controller.current_waypoint)
        # Hold PID working
        pass

    def nop(self, controller: UAVController):
        pass
    
    def move(self, controller: UAVController):
        pass

    def land(self, controller: UAVController):
        controller.land()

    def end(self):
        rospy.signal_shutdown("RACE Task successfully finished! Hooray :-)")