import threading
from Robot import Robot
from src.Visualizer import Visualizer
from src.utils import deg_to_rad

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
                      simulation=sim)
    v = Visualizer(our_robot)

    def cmd():
        our_robot.set_speed(0.15)
        our_robot.move_to_deg([90, -10, 10, 10], True)

        our_robot.set_speed(0.2)
        our_robot.move_to_deg([0, 90, 0, 0], True)

    cmd_thread = threading.Thread(target=cmd)
    cmd_thread.start()

    while cmd_thread.is_alive():
        v.viz()

    our_robot.stop()
