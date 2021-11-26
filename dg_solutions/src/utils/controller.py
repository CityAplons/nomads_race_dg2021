import numpy as np
import time
from threading import Thread

from utils.helpers import NPPose, euclidean_distance

class MavrosControl:
    from geometry_msgs.msg import TwistStamped, PoseStamped
    from nav_msgs.msg import Odometry
    from mavros_msgs.msg import PositionTarget, State
    from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
    import mavros.param as mavros_param
    
    import mavros
    import rospy

    def __init__(self, id: int, params, use_velocity_control: bool = False) -> None:
        self.num = id
        self.mavros.set_namespace(f"/mavros{self.num}")
        if params is not None:
            self.rospy.wait_for_service(f"/mavros{self.num}/param/set")
            self.load_px4_params(params)
        self.state = self.State()
        self.pt = self.PositionTarget()
        self.pt_pose = self.PoseStamped()

        self.local_pose = None
        self.global_pose = None
        self.velocity = np.array([0,0,0], dtype=np.float32)
        
        self.is_velocity = use_velocity_control
        self.subscribe_on_topics()
        self.pub_pt = self.rospy.Publisher(f"/mavros{self.num}/setpoint_raw/local", self.PositionTarget, queue_size=10)
        self.pub_pt_pose = self.rospy.Publisher(f"/mavros{self.num}/setpoint_position/local", self.PoseStamped, queue_size=10)
        self.set_mode_service = self.rospy.ServiceProxy(f"/mavros{self.num}/set_mode", self.SetMode)
    
    def get_position(self):
        return self.local_pose.position
    
    def get_global_position(self):
        return self.global_pose.position

    def get_orientation(self):
        return self.local_pose.orientation
    
    def get_global_orientation(self):
        return self.global_pose.orientation

    def get_target_position(self):
        return np.array([self.pt.position.x, self.pt.position.y, self.pt.position.z])

    def get_target_yaw(self):
        return self.pt.yaw

    def set_mode(self, new_mode):
        if self.state is not None and self.state.mode != new_mode:
            self.service_proxy("set_mode", self.SetMode, custom_mode=new_mode)

    def set_vel(self, velocity):
        if self.is_velocity:
            self.pt.type_mask = (
                self.pt.IGNORE_PX | 
                self.pt.IGNORE_PY | 
                self.pt.IGNORE_PZ | 
                self.pt.IGNORE_AFX | 
                self.pt.IGNORE_AFY | 
                self.pt.IGNORE_AFZ | 
                self.pt.IGNORE_YAW |
                self.pt.IGNORE_YAW_RATE
                )
            self.pt.velocity.x = velocity[0]
            self.pt.velocity.y = velocity[1]
            self.pt.velocity.z = velocity[2]
        else:
            self.rospy.logwarn("Velocity control in position mode")

    def set_yaw_rate(self, yaw_rate):
        if self.is_velocity:
            self.pt.type_mask = (
                self.pt.IGNORE_VX | self.pt.IGNORE_VY | self.pt.IGNORE_VZ |
                self.pt.IGNORE_PX | self.pt.IGNORE_PY | self.pt.IGNORE_PZ | 
                self.pt.IGNORE_AFX | self.pt.IGNORE_AFY | self.pt.IGNORE_AFZ |
                self.pt.IGNORE_YAW
            )
            self.pt.yaw = yaw_rate
        else:
            self.rospy.logwarn("Velocity control in position mode")

    def set_yaw(self, val_deg):
        if not self.is_velocity:
            self.pt.type_mask = (
                self.pt.IGNORE_VX | self.pt.IGNORE_VY | self.pt.IGNORE_VZ |
                self.pt.IGNORE_PX | self.pt.IGNORE_PY | self.pt.IGNORE_PZ | 
                self.pt.IGNORE_AFX | self.pt.IGNORE_AFY | self.pt.IGNORE_AFZ |
                self.pt.IGNORE_YAW_RATE
            )
            self.pt.yaw = val_deg * 0.0174532925
        else:
            self.rospy.logwarn("Position control in velocity mode")

    def set_position(self, position):
        if not self.is_velocity:
            self.pt.type_mask = (
                self.pt.IGNORE_VX | self.pt.IGNORE_VY | self.pt.IGNORE_VZ | 
                self.pt.IGNORE_AFX | self.pt.IGNORE_AFY | self.pt.IGNORE_AFZ | 
                self.pt.IGNORE_YAW_RATE)
                # self.pt.IGNORE_YAW | self.pt.IGNORE_YAW_RATE)
            self.pt.position.x = position[0]
            self.pt.position.y = position[1]
            self.pt.position.z = position[2]
            self.pt.yaw = 0.523599
        else:
            self.rospy.logwarn("Position control in velocity mode")

    def set_pose(self, pose: NPPose):
        if not self.is_velocity:
            self.pt_pose.pose.position.x = pose.position[0]
            self.pt_pose.pose.position.y = pose.position[1]
            self.pt_pose.pose.position.z = pose.position[2]
            # w*e^0 + x*e^1 + y*e^2 + z*e^3
            self.pt_pose.pose.orientation.w = pose.orientation[0]
            self.pt_pose.pose.orientation.x = pose.orientation[1]
            self.pt_pose.pose.orientation.y = pose.orientation[2]
            self.pt_pose.pose.orientation.z = pose.orientation[3]
            
            self.pub_pt_pose(self.pt_pose)
        else:
            self.rospy.logwarn("Position control in velocity mode")

    def service_proxy(self, path, arg_type, *args, **kwds):
        service = self.rospy.ServiceProxy(f"/mavros{self.num}/{path}", arg_type)
        ret = service(*args, **kwds)
    
    def load_px4_params(self, params: dict):
        for param in params.keys():
            while not self.rospy.is_shutdown():
                try:
                    val = self.mavros_param.param_get(param)
                    resp = self.mavros_param.param_set(param, params[param])
                    self.rospy.loginfo("%s value: %f", param, resp)
                except Exception as e:
                    self.rospy.logerr("[%s] Setup failed.", param)
                    print(e)
                    time.sleep(0.5)
                    continue
                break
    
    ### LOCAL_GLOBAL coords switched | hate airsim (me too)
    def gpose_cb(self, msg):
        pose = msg.pose.pose
        self.local_pose = NPPose(pose)

    def pose_cb(self, msg):
        pose = msg.pose
        self.global_pose = NPPose(pose)
    ### end LOCAL_GLOBAL
    
    def velocity_cb(self, msg):
        velocity = msg.twist.linear
        self.velocity = np.array([
            velocity.x, 
            velocity.y, 
            velocity.z
        ])

    def state_cb(self, msg):
        self.state = msg


    def subscribe_on_topics(self):
        self.rospy.Subscriber(f"/mavros{self.num}/global_position/local", self.Odometry, self.gpose_cb)
        self.rospy.Subscriber(f"/mavros{self.num}/local_position/pose", self.PoseStamped, self.pose_cb)
        self.rospy.Subscriber(f"/mavros{self.num}/state", self.State, self.state_cb)
        if self.is_velocity:
            self.rospy.Subscriber(f"/mavros{self.num}/local_position/velocity_local", self.TwistStamped, self.velocity_cb)

class dronePositionController(MavrosControl):

    def __init__(self, id: int, params, pub_rate: int = 20) -> None:
        self.rospy.loginfo("dronePositionController started for instance %d", id)
        super().__init__(id, params, use_velocity_control=False)
        self.drone_states = {
            "ground": self.__nop,
            "arm": self.__st_arm,
            "takeoff": self.__st_takeoff,
            "air": self.__st_pub_pos,
            "land": self.__st_land,
            "disarm": self.__st_disarm
        }
        self.drone_state = "ground"
        self.is_armed = False
        self.is_arrived = False

        self.ignore_sm = False
        self.state_machine = Thread(target=self.loop, args=(pub_rate,))
        self.state_machine.start()

    def loop(self, rate):
        pub_rate = self.rospy.Rate(rate)

        while not self.rospy.is_shutdown():
            self.set_mode("OFFBOARD")
            if self.drone_state in self.drone_states:
                self.drone_states[self.drone_state]()
            pub_rate.sleep()

    def set_arm(self, status: bool = False):
        if status:
            self.drone_state = "arm"
        else:
            self.__st_land()
            self.drone_state = "disarm"

    def takeoff(self) -> bool:
        if self.is_armed:
            self.drone_state = "takeoff"
            return True
        else:
            self.rospy.logwarn("Drone not armed!")
            self.drone_state = "arm"
            return False

    def __st_takeoff(self):
        if self.is_armed and self.global_pose is not None:
            position = self.get_global_position() + np.array([0,0,3.])
            self.set_position(position)
            self.drone_state = "air"

    def __st_land(self):
        if self.state is not None and self.state.armed:
            self.service_proxy("cmd/land", self.CommandTOL)

    def __st_arm(self):
        self.__arming(True)

    def __st_disarm(self):
        self.__arming(False)

    def __st_pub_pos(self):
        d = euclidean_distance(self.get_global_position(), self.get_target_position())
        if d <= 0.6:
            self.is_arrived = True
        else:
            self.is_arrived = False
        if not self.ignore_sm:
            self.pub_pt.publish(self.pt)

    def __arming(self, to_arm):
        self.is_armed = to_arm
        if self.state is not None and self.state.armed != to_arm:
            self.service_proxy("cmd/arming", self.CommandBool, to_arm)

    def __nop(self):
        pass

    def __del__(self):
        self.state_machine.join()