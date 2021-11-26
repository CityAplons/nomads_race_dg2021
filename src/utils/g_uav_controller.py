import rospy
import mavros
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State 
from mavros_msgs.srv import CommandBool, SetMode
from tf.transformations import *
import message_filters

from math import *
import numpy as np
from numpy.linalg import norm
import time

from scipy.spatial.transform import Rotation

class UAVController():
    def __init__(self, num, pid_params: dict = None):
        # Drone id
        self.num = num
        mavros.set_namespace(f"/mavros{self.num}")
        
        self.pose = None
        self.yaw = 0
        self.sp = None
        self.hz = 10
        self.rate = rospy.Rate(self.hz)

        self.current_state = State() # Current autopilot state
        self.prev_request = None
        self.prev_state = None
        self.state = None
        self.current_status = "Init" # 4 states Init, Flight, Hover, Landing, Grounded

        # EKF Pids
        if pid_params is not None:
            rospy.wait_for_service(f"/mavros{self.num}/param/set")
            self.load_pid_params(pid_params)

        self.setpoint_publisher = rospy.Publisher(f"/mavros{self.num}/setpoint_position/local", PoseStamped, queue_size=10)
                
        # ROS services
        service_timeout = 30
        rospy.loginfo("waiting for ROS services")
        try:
            #rospy.wait_for_service('mavros/param/get', service_timeout)
            rospy.wait_for_service(f"/mavros{self.num}/cmd/arming", service_timeout)
            #rospy.wait_for_service('mavros/mission/push', service_timeout)
            #rospy.wait_for_service('mavros/mission/clear', service_timeout)
            rospy.wait_for_service(f"/mavros{self.num}/set_mode", service_timeout)
            rospy.loginfo("ROS services are up")
        except rospy.ROSException:
            self.fail("failed to connect to services")
        self.arming_client = rospy.ServiceProxy(f"/mavros{self.num}/cmd/arming", CommandBool)
        self.set_mode_client = rospy.ServiceProxy(f"/mavros{self.num}/set_mode", SetMode, self.state_callback)
        self.set_mode_service = rospy.ServiceProxy(f"/mavros{self.num}/set_mode", SetMode)

        rospy.Subscriber(f"/mavros{self.num}/state", State, self.state_callback)
        rospy.Subscriber(f"/mavros{self.num}/local_position/pose", PoseStamped, self.drone_pose_callback)
        # rospy.Subscriber(f"/mavros{self.num}/local_position/pose", PoseStamped, self.drone_orientation_callback) # works strange

        # self.setpoint_publisher = rospy.Publisher('uav0/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        # self.arming_client = rospy.ServiceProxy('uav0/mavros/cmd/arming', CommandBool)
        # self.set_mode_client = rospy.ServiceProxy('uav0/mavros/set_mode', SetMode, self.state_callback)
        # rospy.Subscriber('uav0/mavros/state', State, self.state_callback)
        # rospy.Subscriber('uav0/mavros/local_position/pose', PoseStamped, self.drone_pose_callback)

    def load_pid_params(self, params: dict):
        for param in params.keys():
            while True:
                try:
                    val = param_get(param)
                    rospy.loginfo("%s:%f", param, val)
                    resp = param_set(param, params[param])
                    rospy.loginfo("New PX4 %s value: %f", param, resp)
                except:
                    time.sleep(0.5)
                    continue
                break
    
    def state_callback(self, state):
        self.current_state = state

    def drone_pose_callback(self, pose_msg):
        self.pose = np.array([ pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z ])

    # def drone_orientation_callback(self, pose_msg):
    #     self.pose = np.array([ pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w ])

    def arm(self):
        for i in range(self.hz):
            self.publish_setpoint([0,0,-1]) # Z=-1 is a minimum loiter altitude
            self.rate.sleep()
    
        # wait for FCU connection
        while not self.current_state.connected:
            print('Waiting for FCU connection...')
            self.rate.sleep()

        prev_request = rospy.get_time()
        prev_state = self.current_state
        while not rospy.is_shutdown():
            now = rospy.get_time()
            if self.current_state.mode != "OFFBOARD" and (now - prev_request > 2.):
                self.set_mode_client(base_mode=0, custom_mode="OFFBOARD")
                prev_request = now 
            elif not self.current_state.armed and (now - prev_request > 2.):
                   self.arming_client(True)
                   prev_request = now 

            # older versions of PX4 always return success==True, so better to check Status instead
            if prev_state.armed != self.current_state.armed:
                print("Vehicle armed: %r" % self.current_state.armed)

            if prev_state.mode != self.current_state.mode: 
                print("Current mode: %s" % self.current_state.mode)
            print(self.current_state)
            prev_state = self.current_state

            if self.current_state.armed:
                break
            # Update timestamp and publish sp 
            self.publish_setpoint([0,0,-1])
            self.rate.sleep()

    #@staticmethod
    def get_setpoint(self,x, y, z, yaw): # early yaw=np.pi/2
        set_pose = PoseStamped()
        set_pose.pose.position.x = x
        set_pose.pose.position.y = y
        set_pose.pose.position.z = z
        
        # Create a rotation object from Euler angles specifying axes of rotation
        rot = Rotation.from_euler('xyz', [0, 0, yaw], degrees=False)

        # Convert to quaternions and print
        q = rot.as_quat()
        # print('quaternion')
        # print(q)

        # q = quaternion_from_euler(0, 0, yaw)
        set_pose.pose.orientation.x = q[0]
        set_pose.pose.orientation.y = q[1]
        set_pose.pose.orientation.z = q[2]
        set_pose.pose.orientation.w = q[3]
        #print(np.round(self.pose,4),round(yaw,4),self.current_status)
        return set_pose
        
    def publish_setpoint(self, sp, yaw=None): #yaw=np.pi/2
        #yaw = math.atan2(sp[1],sp[0]) if self.current_status == "Hover" else self.yaw
        if (yaw == None): yaw = self.yaw
        #print(np.round(sp,4),round(yaw,4),self.current_status)
        setpoint = self.get_setpoint(sp[0], sp[1], sp[2], yaw)
        setpoint.header.stamp = rospy.Time.now()
        self.setpoint_publisher.publish(setpoint)
        #print("Published...",self.pose)

    # def takeoff(self, height):
    def takeoff(self):
        height=3
        print("Takeoff...",self.pose)
        self.sp = self.pose
        while self.pose[2] < height:
            self.sp[2] += 0.5
            self.publish_setpoint(self.sp)
            self.rate.sleep()
        self.current_status = "Hover"
        print(self.current_status)

    def hover(self, t_hold):
        print('Position holding...')
        t0 = time.time()
        self.sp = self.pose
        while not rospy.is_shutdown():
            t = time.time()
            if t - t0 > t_hold and t_hold > 0: break
            # Update timestamp and publish sp 
            self.publish_setpoint(self.sp)
            self.rate.sleep()
        self.current_status = "Flight"

    def land(self):
        print("Landing...")
        self.current_status = "Landing"
        self.sp = self.pose
        while self.sp[2] > - 1.0:
            self.sp[2] -= 0.05
            self.publish_setpoint(self.sp)
            self.rate.sleep()
        # self.stop()
        self.current_status = "Grounded"
        print(self.current_status)

    def stop(self):
        while self.current_state.armed or self.current_state.mode == "OFFBOARD":
            if self.current_state.armed:
                self.arming_client(False)
            if self.current_state.mode == "OFFBOARD":
                self.set_mode_client(base_mode=0, custom_mode="MANUAL")
            self.rate.sleep()

    @staticmethod
    def transform(pose):
        # transformation: x - forward, y - left, z - up (ENU - MoCap frame)
        pose_new = np.zeros(3)
        pose_new[0] = - pose[1]
        pose_new[1] = pose[0]
        pose_new[2] = pose[2]
        return pose_new

    def goTo(self, wp, mode='global', tol=0.05):
        print("Current setpoint",wp)
        #print(math.atan2(wp[1],wp[0]))
        #wp = self.transform(wp) # it was used for mocap
        yaw = math.atan2(wp[1],wp[0])
        #now = rospy.get_time() # doubt
        if (yaw != self.yaw) and (self.current_status == "Hover"):
            #self.set_mode_client(base_mode=0, custom_mode="ALTCTL") # doubt
            #self.prev_request = now # doubt
            #print("Mode:",SetMode().mode_sent) # doubt
            while norm(yaw - self.yaw) > tol:
                n = (yaw - self.yaw) / norm(yaw - self.yaw)
                self.yaw += 0.03 * n
                self.publish_setpoint(self.pose, self.yaw)
                self.rate.sleep()
            #self.set_mode_client(base_mode=0, custom_mode="OFFBOARD") # doubt
        if (self.current_status == "Init"):
            self.yaw=0
        #if self.current_state.mode != "OFFBOARD": 
        if mode=='global':
            goal = wp
        elif mode=='relative':
            goal = self.pose + wp
        print("Going to a waypoint...")
        self.sp = self.pose
        while norm(goal - self.pose) > tol:
            n = (goal - self.sp) / norm(goal - self.sp)
            self.sp += 0.03 * n
            self.publish_setpoint(self.sp)
            self.rate.sleep()

    def set_mode(self, mode):
        self.set_mode_service(base_mode=0, custom_mode=mode)

        