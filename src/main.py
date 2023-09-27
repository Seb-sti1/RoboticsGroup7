import threading
import time

from Robot import Robot
from src.Visualizer import Visualizer
from src.utils import deg_to_rad, rad_to_deg

if __name__ == '__main__':
    bound_angle_rad = [(deg_to_rad(tup[0]), deg_to_rad(tup[1])) for tup in
                       [(-130, 130), (-180, 0), (-100, 100), (-100, 100)]]

    our_robot = Robot([1, 2, 3, 4],
                      bound_angle_rad,
                      [0.05, 0, 0, 0],
                      [0, 0.093, 0.093, 0.05],
                      [3.14 / 2, 0, 0, 0],
                      simulation=True)
    v = Visualizer(our_robot)

    def cmd():
        our_robot.set_speed(0.1)
        our_robot.move_to_deg([10, -10, -10, -10])
        time.sleep(5)

        our_robot.set_speed(0.5)
        our_robot.move_to_deg([0, 90, 0, 0])
        time.sleep(5)

    cmd_thread = threading.Thread(target=cmd)
    cmd_thread.start()

    while cmd_thread.is_alive():
        v.viz()

    our_robot.stop()
