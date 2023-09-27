"""
An abstraction of the robot. It allows to control the robot.

TODO: add a way to wait till the robot is done moving
"""

import dynamixel_sdk.src.dynamixel_sdk as dxl
from src.Servo import Servo
from src.utils import rad_to_deg, deg_to_rad

BAUDRATE = 1000000  # Baudrate for Motors
DEVICENAME = '/dev/ttyACM0'  # the port of the controller examples: Windows: 'COM1' Linux: '/dev/ttyUSB0'


class Robot:
    def __init__(self, ids, bound_angles, d, a, alpha, port=DEVICENAME, simulation=False):
        """
        :param ids: the ids of the motors
        :param bound_angles: an array of the bound angles of the motors
        :param d: the d parameters of the Denavit Hartenberg table
        :param a: the a parameters of the Denavit Hartenberg table
        :param alpha: the alpha parameters of the Denavit Hartenberg table
        :param port: the port of the controller
        """
        self.dh_parameters = [d, a, alpha]

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
        for motor_id, angle in zip(ids, bound_angles):
            self.servos.append(Servo(motor_id, angle[0], angle[1], self.port_handler, simulation))

    def move_to(self, angles):
        """
        Move the robot to the given angles
        :param angles: an array of angles in radians
        """
        if len(angles) != len(self.servos):
            raise Exception("The number of angles does not match the number of servos")

        for servo, angle in zip(self.servos, angles):
            servo.set_position(angle)

    def move_to_deg(self, angles):
        """
        Move the robot to the given angles
        :param angles: an array of angles in degrees
        """
        self.move_to([deg_to_rad(angle) for angle in angles])

    def get_positions(self):
        """
        Get the current positions of the servos
        """
        return [servo.get_position() for servo in self.servos]

    def set_speed(self, speed):
        """
        Set the speed of the servos
        :param speed: the speed to set
        """
        for servo in self.servos:
            servo.set_speed(speed)

    def draw_robot(self, visualizer, delay):
        """
        Draw the current state of the robot
        """
        thetas = self.get_positions()
        print("Current angles are", [rad_to_deg(theta) for theta in thetas])
        visualizer.show_robot(thetas, self.dh_parameters[0], self.dh_parameters[1], self.dh_parameters[2], delay)

    def stop(self):
        for servo in self.servos:
            servo.disable_motor()

        if self.port_handler is not None:
            self.port_handler.closePort()
