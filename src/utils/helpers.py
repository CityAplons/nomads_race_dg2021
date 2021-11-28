import numpy as np
import math

def euclidean_distance(a, b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 +(a[2]-b[2])**2)

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

class PIDController:
    P = 1.
    I = 0.
    D = 0.
    Tp = 0.
    Ep = 0.

    def __init__(self, P, I, D, limit) -> None:
        self.P = P
        self.I = I
        self.D = D
        self.lim = limit

    def step(self, current, target, timestamp: float):
        if self.Ep is np.ndarray:
            self.Ep = np.zeros(shape=current.shape, dtype=np.float64)

        dt = timestamp - self.Tp
        self.Tp = timestamp

        error = target - current

        integral = self.I * dt * error
        differential = self.D / dt * (error - self.Ep)
        
        self.Ep = error

        updated = (self.P * error) + differential + integral
        updated_norm = np.linalg.norm(updated)
        if updated_norm > self.lim:
           updated = updated / updated_norm * self.lim      
        
        return updated, error

def EulerFromQuaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        source: https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians