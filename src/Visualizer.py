"""
This file contains the Visualizer class which is used to visualize the robot in a 3D plot.
"""
import numpy as np
import matplotlib.pyplot as plt
import matplotlib


def denavit_hartenberg(theta, d, a, alpha):
    """
    Calculate the homogeneous transformation matrix using the DH parameters
    :param theta: the angle between x_i-1 and x_i measured around z_i-1 in radians
    :param d: the distance between z_i-1 and z_i in meters
    :param a: the distance between x_i-1 and x_i in meters
    :param alpha: the angle between z_i-1 and z_i measured around x_i in radians
    :return:
    """
    return np.array([[np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
                     [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
                     [0, np.sin(alpha), np.cos(alpha), d],
                     [0, 0, 0, 1]])


class Frame:
    """
    This class represents a frame in the 3D plot with an origin and x, y, z axis.
    """

    def __init__(self, name, length=0.05):
        self.x = np.array([1, 0, 0])
        self.y = np.array([0, 1, 0])
        self.z = np.array([0, 0, 1])
        self.o = np.array([0, 0, 0])

        self.length = length

    def transform(self, H):
        R = H[:3, :3]
        self.o = H[:3, 3]
        self.x = np.dot(R, self.x)
        self.y = np.dot(R, self.y)
        self.z = np.dot(R, self.z)

    def draw(self, ax):
        for v, c in zip([self.x, self.y, self.z], ['r', 'g', 'b']):
            ax.quiver(self.o[0], self.o[1], self.o[2], v[0], v[1], v[2], color=c, alpha=1., length=self.length,
                      arrow_length_ratio=0.3)


class Visualizer:
    def __init__(self, robot):
        self.robot = robot
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlim([-0.1, 0.3])
        self.ax.set_ylim([-0.3, 0.3])
        self.ax.set_zlim([0, 0.3])
        self.ax.set_xlabel('$x$')
        self.ax.set_ylabel('$y$')
        self.ax.set_zlabel('$z$')

    def show_robot(self, add_to_plot, theta_l, d_l, a_l, alpha_l,  delay):
        """
        Show the robot in a 3D plot
        :param add_to_plot: a function that adds things to the 3D plot
        :param theta_l: the DH parameters
        :param d_l: the DH parameters
        :param a_l: the DH parameters
        :param alpha_l: the DH parameters
        :param delay: the delay that pyplot waits after showing the plot
        :return:
        """
        self.ax.cla()
        self.ax.set_xlim([-0.1, 0.3])
        self.ax.set_ylim([-0.25, 0.25])
        self.ax.set_zlim([-0.08, 0.25])
        self.ax.set_xlabel('$x$')
        self.ax.set_ylabel('$y$')
        self.ax.set_zlabel('$z$')

        frames = [Frame("0")]
        H_pre = np.eye(4)
        for i, (theta, d, a, alpha) in enumerate(zip(theta_l, d_l, a_l, alpha_l)):
            H = np.matmul(H_pre, denavit_hartenberg(theta, d, a, alpha))
            f = Frame(str(i + 1))
            f.transform(H)
            frames.append(f)
            H_pre = H

        # draw the frames
        for f in frames:
            f.draw(self.ax)

        # draw lines between origin of frames
        for i in range(len(frames) - 1):
            self.ax.plot([frames[i].o[0], frames[i + 1].o[0]],
                         [frames[i].o[1], frames[i + 1].o[1]],
                         [frames[i].o[2], frames[i + 1].o[2]],
                         color='k', alpha=1, linewidth=3)

        # add text of the coordinates of the last frame
        self.ax.text(frames[-1].o[0], frames[-1].o[1], frames[-1].o[2],  "({:.3f}, {:.3f}, {:.3f})".format(frames[-1].o[0], frames[-1].o[1], frames[-1].o[2]))

        # run custom function on the plot
        add_to_plot(self.ax)

        plt.draw()
        plt.show(block=False)
        plt.pause(delay)

    def viz(self, delay=0.2):
        """
        Visualize the robot in a 3D plot
        :param delay: the delay that pyplot waits after showing the plot
        :return:
        """
        self.robot.draw_robot(self, delay)
