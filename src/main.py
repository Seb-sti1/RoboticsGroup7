import threading
from math import sin, cos, atan
from time import sleep

import cv2
import numpy as np

from Camera import load_or_create_config, Camera
from Robot import Robot
from RobotDefinition import RoboticsArm
from Visualizer import Visualizer
from utils import deg_to_rad, rad_to_deg

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
        our_robot.move_to_deg([0, 90, 0, 0], wait=True)
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

        step = 20
        default_position = np.array([0, 120, -90, -100])
        search_positions = [default_position + [t0, 0, 0, 0] for t0 in range(int(rad_to_deg(robot_def.get_bound_angle(our_robot)[0][0])),
                                     int(rad_to_deg(robot_def.get_bound_angle(our_robot)[0][1])) + step,
                                     step)]
        search_positions_idx = 0
        lock_search = False

        print("You can quit the program by pressing Echap. When the robot has detected a satisfactory position, "
              "press y to move the robot to the position.")
        our_robot.move_to_deg(search_positions[search_positions_idx], wait=True)
        while True:
            # get the position of the stylus
            ring = None
            while ring is None:
                pitch = deg_to_rad(sum(our_robot.get_servos_angle_not_too_old(100)[1:]))
                ring = c.get_ring_position(pitch)
                if ring is None and lock_search is False:
                    search_positions_idx = (search_positions_idx + 1) % len(search_positions)
                    our_robot.move_to_deg(search_positions[search_positions_idx], wait=True)
            if ring is False:
                print("Exiting")
                break

            # get current robot position
            H05 = np.matmul(our_robot.get_stylus_transformation_matrix(), T45)

            # get ring position in frame 0
            ring_in_zero = np.matmul(H05, np.append(ring, 1))[:3]
            print("Ring position in frame 0:", ring_in_zero)

            if lock_search is False:
                # align the robot with the position of the ring
                our_robot.move_to_deg(default_position + [rad_to_deg(atan(ring_in_zero[1]/ring_in_zero[0])), 0, 0, 0], wait=True)
                # the robot has found the ring, no need to search again (just actualize the position of the ring)
                lock_search = True

            # update the callback to draw the ring
            def draw_ring_center(ax):
                ax.scatter(ring_in_zero[0], ring_in_zero[1], ring_in_zero[2], color="red")
            our_robot.set_custom_plot_draw_callback(draw_ring_center)

            # exit if the user presses q or Echap
            k = cv2.waitKey(30)
            # move the robot to the position if user press y
            if k == ord('y'):
                if our_robot.move_to_pos(ring_in_zero + [0, 0, 0.05], [0, deg_to_rad(-90), 0], wait=True,
                                         verbose=True):
                    print("Position reach. Press enter to continue.")
                k = cv2.waitKey(0)
                while k != 13:
                    k = cv2.waitKey(0)
                # go back to the default position
                search_positions_idx = 0
                lock_search = False


    cmd_thread = threading.Thread(target=cmd)
    cmd_thread.start()

    while cmd_thread.is_alive():
        v.viz(delay=0.5)

    our_robot.stop()
