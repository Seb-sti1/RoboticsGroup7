from Robot import Robot
from src.utils import deg_to_rad

if __name__ == '__main__':
    bound_angle_rad = [(deg_to_rad(tup[0]), deg_to_rad(tup[1])) for tup in
                       [(-130, 130), (-180, 0), (-100, 100), (-100, 100)]]

    our_robot = Robot([1, 2, 3, 4],
                      bound_angle_rad,
                      [0.05, 0, 0, 0],
                      [0, 0.093, 0.093, 0.05],
                      [3.14/2, 0, 0, 0])

    our_robot.move_to_deg([10, -10, -10, -10])
    our_robot.stop()


