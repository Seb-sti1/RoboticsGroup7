import threading
import time

from Robot import Robot
from src.Servo import Servo
from src.Visualizer import Visualizer
from src.utils import deg_to_rad, rad_to_deg
import dynamixel_sdk.src.dynamixel_sdk as dxl

if __name__ == '__main__':

    """
    WARNING: before starting this on the real robot, comment out the following lines the self.set_position(0)
    in Servo class
    """
    sim = True
    port_handler = None

    if not sim:
        port_handler = dxl.PortHandler('/dev/ttyACM0')
        if port_handler.openPort():
            print("Succeeded to open the port")
        else:
            raise Exception("Failed to open the port")

        if port_handler.setBaudRate(1000000):
            print("Succeeded to change the baudrate")
        else:
            raise Exception("Failed to change the baudrate")

    # Unit test 1: Enable servo, set speed, torque and min/max angles
    s = Servo(1, deg_to_rad(-20), deg_to_rad(130),
              lambda theta: theta + deg_to_rad(150),
              lambda angle: angle - deg_to_rad(150),
              port_handler, sim)

    # Unit test 2:
    t1 = s.get_position()
    print(f"Current position {rad_to_deg(t1)}°.")

    print("\033[1mThe position need to correspond to the theta_1 using the DH convention."
          "If it's not the case, the software isn't ready to control the servo!!!\033[0m\n")

    if input("Continue? (Y/n)") != "Y":
        exit(0)

    # Unit test 3: Move to a position
    print("Moving +10° and waiting 5 seconds.")
    s.set_position(deg_to_rad(10 + t1))
    time.sleep(5)
    t1 = s.get_position()
    print(f"Current position {rad_to_deg(t1)}°.\n")
    print("Moving -20° and waiting 10 seconds.")
    s.set_position(deg_to_rad(-20 + t1))
    time.sleep(10)
    print(f"Current position {rad_to_deg(s.get_position())}°.\n")

    print("\033[1mIf the servo is moving, the software is ready to control the servos.\033[0m\n")

    if input("Continue? (Y/n)") != "Y":
        exit(0)

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
                      simulation=True)
    v = Visualizer(our_robot)

    def cmd():
        our_robot.set_speed(0.1)
        our_robot.move_to_deg([10, -10, -10, -10])
        time.sleep(15)

        our_robot.set_speed(0.5)
        our_robot.move_to_deg([0, 90, 0, 0])
        time.sleep(5)

    cmd_thread = threading.Thread(target=cmd)
    cmd_thread.start()

    while cmd_thread.is_alive():
        v.viz()

    our_robot.stop()
