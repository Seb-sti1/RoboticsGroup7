"""
This file test if the reverse kinematics is working correctly by comparing with the forward kinematics
"""
import numpy as np
import math as ma

from matplotlib import pyplot as plt

from RobotDefinition import RoboticsArm
from tqdm import tqdm
from mpi4py import MPI

from utils import deg_to_rad


class FakeRobot:
    def __init__(self):
        self.theta = [0, 0, 0, 0]

    def set_servos_angle(self, theta):
        self.theta = theta

    def get_servos_angle(self):
        return self.theta


def is_egal(a, b, EPS):
    if a > b:
        a, b = b, a

    if a < - ma.pi + EPS and b > ma.pi - EPS:
        return True
    return abs(a - b) % (2 * ma.pi) < EPS


if __name__ == '__main__':
    robot_def = RoboticsArm()
    robot = FakeRobot()

    EPS = 0.0002
    cube_size = 0.3  # m
    cube_center = [0, 0, 0]  # m
    step = 0.05  # m
    angle_step = deg_to_rad(5)  # rad
    show_graph = True

    comm = MPI.COMM_WORLD
    rank = comm.Get_rank()
    size = comm.Get_size()

    comm.barrier()
    if rank == 0:
        reachable = 0
        reachable_points = []
        N = (int(cube_size * 2 / step)) ** 3 * (int(2 * np.pi / angle_step))

        print(f"Testing with x, y, z in [-{cube_size}, {cube_size}] and theta in [-pi, pi] "
              f"with a step of {step} m and {angle_step} rad. (EPS={EPS})")
        print(f"Testing {N} positions with {size} mpi process... (this may take a while)")

        x_l = np.arange(-cube_size, cube_size, step) + cube_center[0]

        for x in (pbar := tqdm(x_l)):
            pbar.set_description(f"[{rank}] Reachable: {reachable} ({reachable / N * 100}%)")
            # find first process that is not busy and send it the x
            dest_rank = -1

            while dest_rank == -1:
                for r in range(1, size):
                    if comm.Iprobe(source=r):
                        dest_rank = r
                        break

            reachable += comm.recv(source=dest_rank)
            if show_graph:
                reachable_points += comm.recv(source=dest_rank)
            comm.send(x, dest=dest_rank)

        # wait for all process to finish
        for r in range(1, size):
            reachable += comm.recv(source=r)
            if show_graph:
                reachable_points += comm.recv(source=r)
            comm.send(False, dest=r)

        print(f"Done. Reachable: {reachable} ({reachable / N * 100}%). If you see this, the test is successful.")

        if show_graph:
            print("Showing graph of reachable position...")
            colors = []

            # gray points are points all accessible by the reverse kinematics model
            # but not by the real robot. For the points accessible by the real robot,
            # the color is proportional to the number of reachable theta
            for points in tqdm(reachable_points):
                if points[3] > 0:
                    c = 255 * points[3] / int(2 * np.pi / angle_step)
                    colors.append("#" + hex(int(c))[2:] + hex(int(c))[2:] + "FF")
                else:
                    colors.append("white")

            reachable_points = np.array(reachable_points)

            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            ax.set_xlim([-0.1, 0.3])
            ax.set_ylim([-0.25, 0.25])
            ax.set_zlim([-0.08, 0.25])
            ax.set_xlabel('$x$')
            ax.set_ylabel('$y$')
            ax.set_zlabel('$z$')
            ax.scatter(reachable_points[:, 0], reachable_points[:, 1], reachable_points[:, 2], s=5, c=colors)
            plt.show()
    else:
        x = None

        reachable = 0
        reachable_points = []

        while True:
            comm.send(reachable, dest=0)
            if show_graph:
                comm.send(reachable_points, dest=0)

            reachable = 0
            reachable_points = []

            x = comm.recv(source=0)
            if x is False:
                break

            for y in np.arange(-cube_size, cube_size, step) + cube_center[1]:
                for z in np.arange(-cube_size, cube_size, step) + cube_center[2]:

                    if show_graph:
                        # the number of reachable theta in bound (of the Dynamixels), and not in bound
                        reachable_theta = [0, 0]

                    for theta in np.arange(-np.pi, np.pi, angle_step):
                        motor_angles = robot_def.reverse_kinematics([x, y, z], [0, theta, 0], EPS=0)
                        if not motor_angles:
                            continue

                        reachable += 1
                        if show_graph:
                            in_bounds = True
                            for bound, motor_angle in zip(robot_def.get_bound_angle(robot), motor_angles):
                                if motor_angle < bound[0] or motor_angle > bound[1]:
                                    in_bounds = False
                                    break

                            if in_bounds:
                                reachable_theta[0] += 1
                            else:
                                reachable_theta[1] += 1

                        robot.set_servos_angle(motor_angles)
                        o, angles = robot_def.forward_kinematics(robot)

                        if (abs(o[0] - x) > EPS or abs(o[1] - y) > EPS
                                or abs(o[2] - z) > EPS or not is_egal(angles[1], theta, EPS)):
                            print(f"[{rank}] Position not equal")
                            print(f"[{rank}] thetas:", motor_angles)
                            print(f"[{rank}] Expected:", x, y, z, theta)
                            print(f"[{rank}] Got:", o[0], o[1], o[2], angles[1])
                            print(f"[{rank}] Diff:", o[0] - x, o[1] - y, o[2] - z, angles[1] - theta)
                            exit(1)

                    if show_graph and sum(reachable_theta) > 0:
                        reachable_points.append([x, y, z, reachable_theta[0], reachable_theta[1]])
