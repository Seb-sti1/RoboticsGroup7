import threading
from math import sin, cos
import numpy as np
from Robot import Robot
from Camera import load_or_create_config, Camera
from RobotDefinition import RoboticsArm
from Visualizer import Visualizer
from utils import deg_to_rad

if __name__ == '__main__':
    sim = True

    port, camera_matrix, color_lower, color_upper = load_or_create_config() if not sim else load_or_create_config(
        "my_webcam_calibration.pkl")

    robot_def = RoboticsArm()
    T45 = robot_def.get_T45()

    our_robot = Robot([1, 2, 3, 4],
                      robot_def,
                      simulation=sim)
    v = Visualizer(our_robot)


    def cmd():
        our_robot.set_speed(0.5)
        # move the robot the full up position
        our_robot.move_to([0, 3.14 / 2, 0, 0], wait=True)
        input("Press enter to start")

        if input("Do you want to do a circle? (Y/n)") == "Y":
            R = 0.032
            pc = [0.15, 0, 0.12]
            N = 36

            for angle in [i * 2 * 3.14 / N for i in range(N + 1)]:
                x = pc[0]
                y = R * cos(angle) + pc[1]
                z = R * sin(angle) + pc[2]
                our_robot.move_to_pos([x, y, z], [0, 0, 0], wait=True)

        print("Loading and starting the config of the camera.")
        c = Camera(camera_matrix, color_lower, color_upper, port)

        print("You can quit the program by pressing Echap.")
        our_robot.move_to([0, deg_to_rad(80), deg_to_rad(-45), deg_to_rad(-105)], wait=True)
        while True:
            # get the position of the stylus
            ring = None
            while ring is None:
                pitch = sum(our_robot.get_servos_angle_not_too_old(100)[1:])
                ring = c.get_ring_position(pitch)
            if ring is False:
                print("Exiting")
                break

            # get robot position
            H05 = np.matmul(our_robot.get_stylus_transformation_matrix(), T45)

            # get ring position in frame 0
            ring_in_zero = np.matmul(H05, np.append(ring, 1))
            print("Ring position in frame 0:", ring_in_zero[:3])

            # update the callback to draw the ring
            def draw_ring_center(ax):
                ax.scatter(ring_in_zero[0], ring_in_zero[1], ring_in_zero[2], color="red")
            our_robot.set_custom_plot_draw_callback(draw_ring_center)

            # move the robot to the position
            our_robot.move_to_pos(ring_in_zero[:3], [0, -3.14/2, 0], wait=True)

            print("Position reach. Press enter to continue.")

    cmd_thread = threading.Thread(target=cmd)
    cmd_thread.start()

    while cmd_thread.is_alive():
        v.viz(delay=0.5)

    our_robot.stop()
