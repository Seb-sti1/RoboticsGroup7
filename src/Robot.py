"""
An abstraction of the robot. It allows to control the robot.
"""
import time

import numpy as np

import dynamixel_sdk.src.dynamixel_sdk as dxl
from Servo import Servo
from utils import rad_to_deg, deg_to_rad

BAUDRATE = 1000000  # Baudrate for Motors
DEVICENAME = '/dev/ttyACM0'  # the port of the controller examples: Windows: 'COM1' Linux: '/dev/ttyUSB0'


class Robot:
    def __init__(self, ids, robot_def,
                 port=DEVICENAME, simulation=False):
        """
        :param ids: the ids of the motors
        :param robot_def: the definition of the robot
        :param port: the port of the controller
        :param simulation: if the robot should be simulated
        """
        self.robot_def = robot_def
        self.port = port
        self.add_to_plot = lambda x: None

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
             theta_to_dxl_angle, dxl_angle_to_theta) in zip(ids, robot_def.get_bound_angle(self),
                                                            robot_def.get_theta_to_dxl_angle_l(self),
                                                            robot_def.get_dxl_angle_to_theta_l(self)):
            self.servos.append(Servo(motor_id,
                                     angle[0], angle[1],
                                     theta_to_dxl_angle, dxl_angle_to_theta,
                                     self.port_handler, simulation))

        self.last_angles = (self.get_servos_angle(), time.time_ns())

    def set_custom_plot_draw_callback(self, add_to_plot):
        """
        Set a callback function that the visualizer will call when drawing the robot
        :param add_to_plot: (plt_ax) -> None
        """
        self.add_to_plot = add_to_plot

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

    def move_to_pos(self, o, angle, wait=False, throw=False, verbose=False):
        """
        Move the robot to the given position
        :param o: x, y, z
        :param angle: roll, pitch, yaw
        :param wait: if the function should wait till the robot is done moving
        :param throw: throw an exception is the position is not reachable
        :return:
        """

        angles = self.robot_def.reverse_kinematics(o, angle, self, verbose=verbose)
        if angles:
            in_bounds = True
            for angle, servo in zip(angles, self.servos):
                if angle < servo.bound[0] or angle > servo.bound[1]:
                    if verbose:
                        print(f"Angle {rad_to_deg(angle)} out of bounds for servo {servo.motor_id} ({servo.bound}).")
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

    def get_servos_angle(self):
        """
        Get the current positions of the servos
        """
        return [servo.get_position() for servo in self.servos]

    def get_servos_angle_not_too_old(self, delay):
        """
        Get the last saved positions of the servos if the last save is older than the delay

        :param delay: the delay in microseconds
        """
        if time.time_ns() - self.last_angles[1] > delay * 1000:
            self.last_angles = (self.get_servos_angle(), time.time_ns())
        return self.last_angles[0]

    def get_stylus_transformation_matrix(self):
        return self.robot_def.get_stylus_transformation_matrix(self)

    def get_stylus_position(self):
        H = self.get_stylus_transformation_matrix()
        return H[:3, 3]

    def is_moving(self, delay=0.1):
        """
        Check if the robot is moving
        :param delay: the delay between the checks
        :return: boolean
        """
        positions_before = self.get_servos_angle()
        time.sleep(delay)
        positions_after = self.get_servos_angle()
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
        thetas = self.robot_def.get_theta(self)
        if log:
            print("Current angles are", [rad_to_deg(theta) for theta in thetas])
        visualizer.show_robot(self.add_to_plot,
                              thetas,
                              self.robot_def.get_d(self),
                              self.robot_def.get_a(self),
                              self.robot_def.get_alpha(self),
                              delay)

    def stop(self):
        for servo in self.servos:
            servo.disable_motor()

        if self.port_handler is not None:
            self.port_handler.closePort()
