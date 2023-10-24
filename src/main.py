import threading
import time
from math import atan2, sin, cos, acos

from Robot import Robot
from src.Visualizer import Visualizer
from src.utils import deg_to_rad


def reverse_kinematics(o, angles, d, a, alpha):
    """
    Find the parameters of the motors given the position of the end effector
    :param o: vector of x, y, z
    :param angles: vector roll pitch yaw
    :param d:
    :param a:
    :param alpha:
    :return: the angles of the different motors or false if the position is not reachable
    """
    x, y, z = o
    # phi = 0 and psi is ignored
    phi, theta, psi = angles

    if phi != 0:
        return False

    theta_1 = atan2(y, x)
    # temp vars
    mu = x/cos(theta_1) - a[3]*cos(theta)
    rho = z - d[0] - a[3]*sin(theta)

    cos_theta_3 = (rho**2 + mu**2 - a[1]**2 - a[2]**2) / (2*a[1]*a[2])
    if abs(cos_theta_3) > 1:
        print(f"\\cos(\\theta_3) is outside of [-1, 1] by {abs(cos_theta_3) - 1}")
        if abs(cos_theta_3) > 1.01:
            return False
        else:
            print("Clamping the value to -1 or 1")
            cos_theta_3 = 1 if cos_theta_3 > 0 else -1

    theta_3 = -acos(cos_theta_3)
    theta_2 = atan2((-a[2]*sin(theta_3)*mu + (a[1] + a[2]*cos(theta_3))*rho),
                    ((a[1] + a[2]*cos(theta_3))*mu + a[2]*sin(theta_3)*rho))
    theta_4 = theta - theta_2 - theta_3

    return [theta_1, theta_2, theta_3, theta_4]


if __name__ == '__main__':
    sim = True

    bound_angle_rad = [(deg_to_rad(tup[0]), deg_to_rad(tup[1])) for tup in
                       [(-130, 130), (-30, 180), (-100, 100), (-100, 100)]]

    our_robot = Robot([1, 2, 3, 4],
                      bound_angle_rad,
                      [0.05, 0, 0, 0],
                      [0, 0.093, 0.093, 0.05],
                      [3.14 / 2, 0, 0, 0],
                      [lambda theta: theta + deg_to_rad(150),
                       lambda theta: theta + deg_to_rad(60),
                       lambda theta: theta + deg_to_rad(150),
                       lambda theta: theta + deg_to_rad(150)],
                      [lambda angle: angle - deg_to_rad(150),
                       lambda angle: angle - deg_to_rad(60),
                       lambda angle: angle - deg_to_rad(150),
                       lambda angle: angle - deg_to_rad(150)],
                      reverse_kinematics,
                      simulation=sim)
    v = Visualizer(our_robot)

    def cmd():
        our_robot.set_speed(0.15)
        our_robot.move_to_pos([0.2, 0.1, 0.1], [0, 0, 0], wait=True)

        input("Press enter to finish")

    cmd_thread = threading.Thread(target=cmd)
    cmd_thread.start()

    while cmd_thread.is_alive():
        v.viz()

    our_robot.stop()
