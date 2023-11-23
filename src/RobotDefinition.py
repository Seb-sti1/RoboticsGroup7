"""
This file contains the definition of the robot.
"""
from math import atan2, cos, sin, acos, sqrt
import numpy as np

from Visualizer import denavit_hartenberg
from utils import deg_to_rad


class RobotDefinition:
    """
    This in an abstract class to define the robot. It is used to define the parameters of the robot
    and the reverse kinematics
    """

    def reverse_kinematics(self, o, angles, robot=None):
        pass

    def get_d(self, robot=None):
        pass

    def get_a(self, robot=None):
        pass

    def get_alpha(self, robot=None):
        pass

    def get_theta(self, robot=None):
        pass

    def get_theta_to_dxl_angle_l(self, robot=None):
        pass

    def get_dxl_angle_to_theta_l(self, robot=None):
        pass

    def get_bound_angle(self, robot=None):
        pass


class RoboticsArm(RobotDefinition):

    def get_theta(self, robot=None):
        return robot.get_servos_angle()

    def get_d(self, robot=None):
        return [0.05, 0, 0, 0]

    def get_a(self, robot=None):
        return [0, 0.093, 0.093, 0.05]

    def get_alpha(self, robot=None):
        return [3.14 / 2, 0, 0, 0]

    def get_stylus_transformation_matrix(self, robot=None):
        H = np.eye(4)
        for i, (theta, d, a, alpha) in enumerate(zip(self.get_theta(robot),
                                                     self.get_d(robot),
                                                     self.get_a(robot),
                                                     self.get_alpha(robot))):
            H = np.matmul(H, denavit_hartenberg(theta, d, a, alpha))
        return H

    def forward_kinematics(self, robot=None):
        H = self.get_stylus_transformation_matrix(robot)

        roll = 0
        pitch = atan2(H[2, 0], H[2, 1])
        yaw = atan2(H[0, 2], -H[1, 2])

        return H[:3, 3], [roll, pitch, yaw]

    def reverse_kinematics(self, o, angles, robot=None, verbose=False, EPS=0.0001):
        """
        Find the parameters of the motors given the position of the end effector
        :param o: vector of x, y, z
        :param angles: vector roll pitch yaw
        :param robot: the robot
        :param verbose: if the function should print the errors
        :param EPS: the error margin
        :return: the angles of the different motors or false if the position is not reachable
        """
        x, y, z = o
        # phi = 0 and psi is ignored
        phi, theta, psi = angles
        d, a = self.get_d(robot), self.get_a(robot)

        if phi != 0:
            return False

        theta_1 = atan2(y, x)
        # temp vars
        mu = sqrt(x**2 + y**2) - a[3] * cos(theta)
        rho = z - d[0] - a[3] * sin(theta)

        cos_theta_3 = (rho ** 2 + mu ** 2 - a[1] ** 2 - a[2] ** 2) / (2 * a[1] * a[2])
        if abs(cos_theta_3) > 1:
            if verbose:
                print(f"\\cos(\\theta_3) is outside of [-1, 1] by {abs(cos_theta_3) - 1}")
            if abs(cos_theta_3) > 1 + EPS:
                return False
            else:
                if verbose:
                    print("Clamping the value to -1 or 1")
                cos_theta_3 = 1 if cos_theta_3 > 0 else -1

        theta_3 = -acos(cos_theta_3)
        theta_2 = atan2((-a[2] * sin(theta_3) * mu + (a[1] + a[2] * cos(theta_3)) * rho),
                        ((a[1] + a[2] * cos(theta_3)) * mu + a[2] * sin(theta_3) * rho))
        theta_4 = theta - theta_2 - theta_3

        return [theta_1, theta_2, theta_3, theta_4]

    def get_theta_to_dxl_angle_l(self, robot=None):
        return [lambda theta: theta + deg_to_rad(150),
                lambda theta: theta + deg_to_rad(60),
                lambda theta: theta + deg_to_rad(150),
                lambda theta: theta + deg_to_rad(240)]

    def get_dxl_angle_to_theta_l(self, robot=None):
        return [lambda angle: angle - deg_to_rad(150),
                lambda angle: angle - deg_to_rad(60),
                lambda angle: angle - deg_to_rad(150),
                lambda angle: angle - deg_to_rad(240)]

    def get_bound_angle(self, robot=None):
        return [(deg_to_rad(tup[0]), deg_to_rad(tup[1])) for tup in
                [(-130, 130), (-20, 170), (-130, 120), (-105, 50)]]

    def get_T45(self):
        return np.array([[1, 0, 0, 0.045],
                         [0, 1, 0, -0.015],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1]])
