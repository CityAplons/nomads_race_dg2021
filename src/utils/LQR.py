#PRAGMA: DOES NOT WORK

from math import modf
import numpy as np
import scipy
from utils.helpers import *

np.random.seed(43)
np.seterr(all='raise')

# 3D Control of Quadcopter
# based on https://github.com/juanmed/quadrotor_sim/blob/master/3D_Quadrotor/3D_control_with_body_drag.py
# The dynamics is from pp. 17, Eq. (2.22). https://www.kth.se/polopoly_fs/1.588039.1550155544!/Thesis%20KTH%20-%20Francesco%20Sabatino.pdf
# The linearization is from Different Linearization Control Techniques for
# a Quadrotor System (many typos)

class DroneDynamics:
    """Qudrotor system dynanmics
    
    Ref:    FRANCESCO SABATINO, "Quadrotor control: modeling, nonlinear control design, and simulation"
            https://www.kth.se/polopoly_fs/1.588039.1600688317!/Thesis%20KTH%20-%20Francesco%20Sabatino.pdf
            (Eq. 2.27)
    """

    def __init__(self) -> None:
        # quadrotor physical constants
        self.g = 9.81
        self.m = 1000
        self.Ix = 8.1 * 1e-1
        self.Iy = 8.1 * 1e-1
        self.Iz = 14.2 * 1e-1

    def check_angle(self, angle):
        frac, _ = modf(angle/np.pi)
        return frac*np.pi

    def f(self, x, u):
        print(x,u)
        x1, x2, y1, y2, z1, z2, phi1, phi2, theta1, theta2, psi1, psi2 = x.reshape(-1).tolist()
        ft, tau_x, tau_y, tau_z = u.reshape(-1).tolist()
        phi1 = self.check_angle(phi1)
        theta1 = self.check_angle(theta1)
        psi1 = self.check_angle(psi1)
        dot_x = np.array([
            x2,
            ft/self.m*(np.sin(phi1)*np.sin(psi1)+np.cos(phi1)*np.cos(psi1)*np.sin(theta1)),
            y2,
            ft/self.m*(np.cos(phi1)*np.sin(psi1)*np.sin(theta1)-np.cos(psi1)*np.sin(phi1)),
            z2,
            -self.g+ft/self.m*np.cos(phi1)*np.cos(theta1),
            phi2,
            (self.Iy-self.Iz)/self.Ix*theta2*psi2+tau_x/self.Ix,
            theta2,
            (self.Iz-self.Ix)/self.Iy*phi2*psi2+tau_y/self.Iy,
            psi2,
            (self.Ix-self.Iy)/self.Iz*phi2*theta2+tau_z/self.Iz
        ])
        return dot_x

class LQRController:

    dynamics = DroneDynamics()
    # X subsystem
    # The state variables are x, dot_x, pitch, dot_pitch
    Ax = np.array(
        [[0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, dynamics.g, 0.0],
        [0.0, 0.0, 0.0, 1.0],
        [0.0, 0.0, 0.0, 0.0]])
    Bx = np.array(
        [[0.0],
        [0.0],
        [0.0],
        [1 / dynamics.Ix]])

        # Y subsystem
        # The state variables are y, dot_y, roll, dot_roll
    Ay = np.array(
        [[0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, -dynamics.g, 0.0],
        [0.0, 0.0, 0.0, 1.0],
        [0.0, 0.0, 0.0, 0.0]])
    By = np.array(
        [[0.0],
        [0.0],
        [0.0],
        [1 / dynamics.Iy]])

    # Z-subsystem
    # The state variables are z, dot_z
    Az = np.array(
        [[0.0, 1.0],
        [0.0, 0.0]])
    Bz = np.array(
        [[0.0],
        [1 / dynamics.m]])

    # Yaw-subsystem
    # The state variables are yaw, dot_yaw
    Ayaw = np.array(
        [[0.0, 1.0],
        [0.0, 0.0]])
    Byaw = np.array(
        [[0.0],
        [1 / dynamics.Iz]])

    def __init__(self, as_linear: bool = True) -> None:
        self.linear = as_linear
        self.last_x = np.zeros(12)
    
    def __cl_linear(s, x, u):
        # closed-loop dynamics. u should be a function
        x = np.array(x)
        X, Y, Z, Yaw = x[[0, 1, 8, 9]], x[[2, 3, 6, 7]], x[[4, 5]], x[[10, 11]]
        UZ, UY, UX, UYaw = u(x).reshape(-1).tolist()
        dot_X = s.Ax.dot(X) + (s.Bx * UX).reshape(-1)
        dot_Y = s.Ay.dot(Y) + (s.By * UY).reshape(-1)
        dot_Z = s.Az.dot(Z) + (s.Bz * UZ).reshape(-1)
        dot_Yaw = s.Ayaw.dot(Yaw) + (s.Byaw * UYaw).reshape(-1)
        dot_x = np.concatenate(
            [dot_X[[0, 1]], dot_Y[[0, 1]], dot_Z, dot_Y[[2, 3]], dot_X[[2, 3]], dot_Yaw])
        return dot_x

    def __cl_nonlinear(s, x, u):
        x = np.array(x)
        dot_x = s.dynamics.f(x, u(x) + np.array([s.dynamics.m * s.dynamics.g, 0, 0, 0]))
        return dot_x

    def lqr(s, A, B, Q, R):
        """Solve the continuous time lqr controller.
            
            dx/dt = A x + B u
            cost = integral x.T*Q*x + u.T*R*u
        """
        # First, try to solve the ricatti equation
        X = np.matrix(scipy.linalg.solve_continuous_are(A, B, Q, R))
        # Compute the LQR gain
        K = np.matrix(scipy.linalg.inv(R) * (B.T * X))
        eigVals, eigVecs = scipy.linalg.eig(A - B * K)

        return np.asarray(K), np.asarray(X), np.asarray(eigVals)

    def solve(s):
        ### Solve LQR
        Ks = []  # feedback gain matrices K for each subsystem
        for A, B in ((s.Ax, s.Bx), (s.Ay, s.By), (s.Az, s.Bz), (s.Ayaw, s.Byaw)):
            n = A.shape[0]
            m = B.shape[1]
            Q = np.eye(n)
            Q[0, 0] = 10.  # The first state variable is the one we care about.
            R = np.diag([1., ])
            K, _, _ = s.lqr(A, B, Q, R)
            Ks.append(K)
        return Ks

    def step(s, target_point, target_yaw):
        Ks = s.solve()

        def u(x):
            # The controller
            UX = Ks[0].dot(np.array([target_point[0], 0, 0, 0]) - x[[0, 1, 8, 9]])[0]
            UY = Ks[1].dot(np.array([target_point[1], 0, 0, 0]) - x[[2, 3, 6, 7]])[0]
            UZ = Ks[2].dot(np.array([target_point[2], 0]) - x[[4, 5]])[0]
            UYaw = Ks[3].dot(np.array([target_yaw, 0]) - x[[10, 11]])[0]
            return np.array([UZ, UY, UX, UYaw])

        x = None
        if s.linear:
            x = s.__cl_linear(s.last_x,u)
        else:
            x = s.__cl_nonlinear(s.last_x,u)

        s.last_x = x
        return np.array([x[0], x[2], x[4]]), x[10]