"""
This is an abstraction of the dynamixel motor. It allows to control the motor.
"""
import dynamixel_sdk.src.dynamixel_sdk as dxl
import ControlTableAddress as addr
from utils import rad_to_rot, rot_to_rad


class Servo:
    def __init__(self, motor_id, min_angle, max_angle, port_handler):
        """
        :param motor_id: the motor id
        :param min_angle: the min angle in radians
        :param max_angle: the max angle in radians
        :param port_handler: the port handler
        """

        self.motor_id = motor_id
        # TODO send constraint to the motor https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/#cwccw-angle-limit6-8
        self.bound = [min_angle, max_angle]
        self.port_handler = port_handler

        self.packet_handler = dxl.PacketHandler(addr.PROTOCOL_VERSION)

        self.enable_motor()
        self.set_position(0)
        self.set_speed(1.3)
        self.set_torque_limit(1)

    def __write1ByteTxRx__(self, address, data):
        r, e = self.packet_handler.write1ByteTxRx(self.port_handler, self.motor_id, address, data)
        return self.check_comm_result(r, e)

    def __write2ByteTxRx__(self, address, data):
        r, e = self.packet_handler.write2ByteTxRx(self.port_handler, self.motor_id, address, data)
        return self.check_comm_result(r, e)

    def check_comm_result(self, dxl_comm_result, dxl_error):
        """
        Check the last communication result
        :return: if the communication was successful
        """
        if dxl_comm_result != addr.COMM_SUCCESS:
            print(f'[{self.motor_id}] {self.packet_handler.getTxRxResult(dxl_comm_result)}')
        elif dxl_error != 0:
            print(f'[{self.motor_id}] {self.packet_handler.getRxPacketError(dxl_error)}')
        else:
            return True
        return False

    def get_position(self):
        """
        :return: the position of the motor in radians
        """
        dxl_present_position, r, e = self.packet_handler.read2ByteTxRx(self.port_handler,
                                                                       self.motor_id,
                                                                       addr.ADDR_MX_PRESENT_POSITION)
        self.check_comm_result(r, e)
        return rot_to_rad(dxl_present_position)

    def set_position(self, angle):
        """
        Move the servo to the given angle
        :param angle: the angle to move to (in radians)
        :return: if the angle is between the bounds
        """

        if angle < self.bound[0] or self.bound[1] < angle:
            print(f"[{self.motor_id}] Warning: Angle out of range,"
                  f" value clamp between [{self.bound[0]}, {self.bound[1]}]")

        return self.__write2ByteTxRx__(addr.ADDR_MX_GOAL_POSITION, int(rad_to_rot(angle)))

    def set_speed(self, speed):
        """
        Set the speed of the servo

        :param speed: the speed in rad / s. max is 11.9380496 rad/s
        :return:
        """
        if speed < 0 or speed > 11.9380496:
            print(f"[{self.motor_id}] Warning: Speed out of range, value clamp between [0, 11.9380496]")
        speed = max(0, min(int(speed / 11.9380496 * 1023), 1023))

        self.__write2ByteTxRx__(addr.ADDR_MX_MOVING_SPEED, speed)

    def set_torque_limit(self, torque):
        """
        Set the torque limit of the servo

        :param torque: between 0 and 1. A percentage of the max torque.
        :return:
        """

        if torque < 0 or torque > 1:
            print(f"[{self.motor_id}] Warning: Torque out of range, value clamp between [0,1]")
            return
        torque = max(0, min(int(torque * 1023), 1023))

        self.__write2ByteTxRx__(addr.ADDR_MX_TORQUE_LIMIT, torque)

    def enable_motor(self):
        """
        Enable the motor
        :return:
        """
        result = self.__write1ByteTxRx__(addr.ADDR_MX_TORQUE_ENABLE, addr.TORQUE_ENABLE)
        self.__write1ByteTxRx__(addr.ADDR_MX_CW_COMPLIANCE_MARGIN, 0)
        self.__write1ByteTxRx__(addr.ADDR_MX_CCW_COMPLIANCE_MARGIN, 0)
        self.__write1ByteTxRx__(addr.ADDR_MX_CW_COMPLIANCE_SLOPE, 32)
        self.__write1ByteTxRx__(addr.ADDR_MX_CCW_COMPLIANCE_SLOPE, 32)

        if result:
            print(f"[{self.motor_id}] Connected, torque mode enabled")
        else:
            print(f"[{self.motor_id}] Connection failed, torque mode not enable")

    def disable_motor(self):
        """
        Disable the motor
        """
        if self.__write1ByteTxRx__(addr.ADDR_MX_TORQUE_ENABLE, addr.TORQUE_DISABLE):
            print(f"[{self.motor_id}] Disconnected, torque mode disabled")
