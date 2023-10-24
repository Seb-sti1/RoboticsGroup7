"""
An abstraction of the robot. It allows to control the robot.
"""
import time
import dynamixel_sdk.src.dynamixel_sdk as dxl
from src.Servo import Servo
from src.utils import rad_to_deg, deg_to_rad

BAUDRATE = 1000000  # Baudrate for Motors
DEVICENAME = '/dev/ttyACM0'  # the port of the controller examples: Windows: 'COM1' Linux: '/dev/ttyUSB0'


class Robot:
    def __init__(self, ids,
                 bound_angles, d, a, alpha,
                 theta_to_dxl_angle_l, dxl_angle_to_theta_l,
                 reverse_kinematics,
                 port=DEVICENAME, simulation=False):
        """
        :param ids: the ids of the motors
        :param bound_angles: an array of the bound angles of the motors
        :param d: the d parameters of the Denavit Hartenberg table
        :param a: the a parameters of the Denavit Hartenberg table
        :param alpha: the alpha parameters of the Denavit Hartenberg table
        :param theta_to_dxl_angle_l: a list of functions to convert theta_i to the angle using the dynamixel convention
        :param dxl_angle_to_theta_l: a list of functions to convert the angle using the dynamixel convention to theta_i
        :param reverse_kinematics: a function to calculate the angles of the motors given the position of the end effector
        :param port: the port of the controller
        :param simulation: if the robot should be simulated
        """
        self.dh_parameters = [d, a, alpha]
        self.reverse_kinematics = reverse_kinematics

        self.port = port

        if not simulation:
            self.port_handler = dxl.PortHandler(self.port)
            if self.port_handler.openPort():
                print("Succeeded to open the port")
            else:
                raise Exception("Failed to open the port")

            if self.port_handler.setBaudRate(BAUDRATE):
                print("Succeeded to change the baudrate")
            else:
                raise Exception("Failed to change the baudrate")
        else:
            self.port_handler = None

        # create the servos
        self.servos = []
        for (motor_id, angle,
             theta_to_dxl_angle, dxl_angle_to_theta) in zip(ids, bound_angles,
                                                            theta_to_dxl_angle_l, dxl_angle_to_theta_l):
            self.servos.append(Servo(motor_id,
                                     angle[0], angle[1],
                                     theta_to_dxl_angle, dxl_angle_to_theta,
                                     self.port_handler, simulation))

    def move_to(self, angles, wait=False):
        """
        Move the robot to the given angles
        :param angles: an array of angles in radians
        :param wait: if the function should wait till the robot is done moving
        """
        if len(angles) != len(self.servos):
            raise Exception("The number of angles does not match the number of servos")

        for servo, angle in zip(self.servos, angles):
            servo.set_position(angle)

        if wait:
            while self.is_moving():
                pass

    def move_to_deg(self, angles, wait=False):
        """
        Move the robot to the given angles
        :param angles: an array of angles in degrees
        :param wait: if the function should wait till the robot is done moving
        """
        self.move_to([deg_to_rad(angle) for angle in angles], wait)

    def move_to_pos(self, o, angle, wait=False, throw=False):
        """
        Move the robot to the given position
        :param o: x, y, z
        :param angle: roll, pitch, yaw
        :param wait: if the function should wait till the robot is done moving
        :param throw: throw an exception is the position is not reachable
        :return:
        """

        angles = self.reverse_kinematics(o, angle, self.dh_parameters[0], self.dh_parameters[1], self.dh_parameters[2])
        if angles:
            in_bounds = True
            for angle, servo in zip(angles, self.servos):
                if angle < servo.bound[0] or angle > servo.bound[1]:
                    in_bounds = False
                    break
            if in_bounds:
                self.move_to(angles, wait)
                return True

        if throw:
            raise Exception("Position not reachable")
        else:
            print("Position not reachable")

        return False

    def get_positions(self):
        """
        Get the current positions of the servos
        """
        return [servo.get_position() for servo in self.servos]

    def is_moving(self, delay=0.1):
        """
        Check if the robot is moving
        :param delay: the delay between the checks
        :return: boolean
        """
        positions_before = self.get_positions()
        time.sleep(delay)
        positions_after = self.get_positions()
        diff = 0
        for position_before, position_after in zip(positions_before, positions_after):
            diff += abs(position_after - position_before)
        return diff > 0.001

    def set_speed(self, speed):
        """
        Set the speed of the servos
        :param speed: the speed to set
        """
        for servo in self.servos:
            servo.set_speed(speed)

    def draw_robot(self, visualizer, delay, log=False):
        """
        Draw the current state of the robot
        """
        thetas = self.get_positions()
        if log:
            print("Current angles are", [rad_to_deg(theta) for theta in thetas])
        visualizer.show_robot(thetas, self.dh_parameters[0], self.dh_parameters[1], self.dh_parameters[2], delay)

    def stop(self):
        for servo in self.servos:
            servo.disable_motor()

        if self.port_handler is not None:
            self.port_handler.closePort()
