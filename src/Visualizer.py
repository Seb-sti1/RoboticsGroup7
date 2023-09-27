import numpy as np
import matplotlib.pyplot as plt
import matplotlib

matplotlib.interactive(False)


def denavit_hartenberg(theta, d, a, alpha):
    return np.array([[np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
                     [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
                     [0, np.sin(alpha), np.cos(alpha), d],
                     [0, 0, 0, 1]])


def draw(ax, origin, vectors, colors, a, l):
    for i in range(len(vectors)):
        v = vectors[i]
        c = colors[i]
        ax.quiver(origin[0], origin[1], origin[2], v[0], v[1], v[2], color=c, alpha=a, length=l, arrow_length_ratio=0.3)


class Frame:
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
        draw(ax, self.o, [self.x, self.y, self.z], ['r', 'g', 'b'], 1., self.length)


def rotation_matrix(angle, rot_axis, x, y, z):
    if rot_axis == "x":
        return np.array([[1, 0, 0, x],
                         [0, np.cos(angle), -np.sin(angle), y],
                         [0, np.sin(angle), np.cos(angle), z],
                         [0, 0, 0, 1]])
    elif rot_axis == "y":
        return np.array([[np.cos(angle), 0, np.sin(angle), x],
                         [0, 1, 0, y],
                         [-np.sin(angle), 0, np.cos(angle), z],
                         [0, 0, 0, 1]])
    else:
        return np.array([[np.cos(angle), -np.sin(angle), 0, x],
                         [np.sin(angle), np.cos(angle), 0, y],
                         [0, 0, 1, z],
                         [0, 0, 0, 1]])


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

    def show_robot(self, theta_l, d_l, a_l, alpha_l, delay):
        self.ax.cla()
        self.ax.set_xlim([-0.1, 0.3])
        self.ax.set_ylim([-0.25, 0.25])
        self.ax.set_zlim([0, 0.25])
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

        plt.draw()
        plt.show(block=False)
        plt.pause(delay)

    def viz(self, delay=0.2):
        self.robot.draw_robot(self, delay)

