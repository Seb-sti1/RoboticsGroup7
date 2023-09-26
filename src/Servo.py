import dynamixel_sdk.src.dynamixel_sdk as dxl
import ControlTableAddress as addr


def deg_to_rot(deg):
    """
    :param deg: the angle in degrees
    :return: value in units per rotation of motor
    """
    return deg * 1 / 0.29


def rot_to_deg(rot):
    """
    :param rot: value in units per rotation of motor
    :return: the angle in degrees
    """
    return rot * 0.29


def rad_to_deg(rad):
    """
    :param rad: the angle in radians
    :return: the angle in degrees
    """
    return rad * 180 / 3.141592


def deg_to_rad(deg):
    """
    :param deg: the angle in degrees
    :return: the angle in radians
    """
    return deg * 3.141592 / 180


def rad_to_rot(rad):
    """
    :param rad: the angle in radians
    :return: value in units per rotation of motor
    """
    return deg_to_rot(rad_to_deg(rad))


def rot_to_rad(rot):
    """
    :param rot: value in units per rotation of motor
    :return: the angle in radians
    """
    return deg_to_rad(rot_to_deg(rot))


class Servo:
    def __init__(self, motor_id, min_angle, max_angle, port_handler):
        """

        :param motor_id: the motor id
        :param min_angle: the min angle in radians
        :param max_angle: the max angle in radians
        """

        self.motor_id = motor_id
        self.bound = [min_angle, max_angle]
        self.port_handler = port_handler

        self.packet_handler = dxl.PacketHandler(addr.PROTOCOL_VERSION)

        self.enable_motor()
        self.set_speed(0.1)
        self.set_torque_limit(1)

    def __write1ByteTxRx__(self, address, data):
        return self.packet_handler.write1ByteTxRx(self.port_handler, self.motor_id, address, data)

    def __write2ByteTxRx__(self, address, data):
        return self.packet_handler.write2ByteTxRx(self.port_handler, self.motor_id, address, data)

    def check_comm_result(self):
        """
        Check the last communication result
        :return: if the communication was successful
        """
        dxl_comm_result = self.packet_handler.getLastTxRxResult(self.port_handler, addr.PROTOCOL_VERSION)
        dxl_error = self.packet_handler.getLastRxPacketError(self.port_handler, addr.PROTOCOL_VERSION)
        if dxl_comm_result != addr.COMM_SUCCESS:
            print('\n#s', self.packet_handler.getTxRxResult(addr.PROTOCOL_VERSION, dxl_comm_result))
        elif dxl_error != 0:
            print('\n#s', self.packet_handler.getRxPacketError(addr.PROTOCOL_VERSION, dxl_error))
        else:
            return True
        return False

    def get_position(self):
        """
        :return: the position of the motor
        """
        dxl_present_position = self.packet_handler.read2ByteTxRx(self.port_handler,
                                                                 self.motor_id,
                                                                 addr.ADDR_MX_PRESENT_POSITION)
        self.check_comm_result()
        return rot_to_deg(dxl_present_position)

    def set_position(self, angle):
        """
        Move the servo to the given angle
        :param angle: the angle to move to (in radians)
        :return: if the angle is between the bounds
        """

        if self.bound[0] < angle < self.bound[1]:
            self.__write2ByteTxRx__(addr.ADDR_MX_GOAL_POSITION, rad_to_rot(angle))
            return True

        return False

    def set_speed(self, speed):
        """
        Set the speed of the servo

        TODO find the units of speed
        :param speed: between 0 and 1
        :return:
        """

        if speed < 0 or speed >= 1:
            print("\nMovement speed out of range, enter value between ]0,1]")
            return

        self.__write2ByteTxRx__(addr.ADDR_MX_MOVING_SPEED, speed * 1023)
        self.check_comm_result()

    def set_torque_limit(self, torque):
        """
        Set the torque limit of the servo

        TODO find the units of torque
        :param torque: between 0 and 1
        :return:
        """

        if torque < 0 or torque >= 1:
            print("\nTorque limit out of range, enter value between ]0,1]")
            return

        self.__write2ByteTxRx__(addr.ADDR_MX_TORQUE_LIMIT, torque * 1023)
        self.check_comm_result()

    def enable_motor(self):
        """
        Enable the motor
        :return:
        """
        self.__write1ByteTxRx__(addr.ADDR_MX_TORQUE_ENABLE, addr.TORQUE_ENABLE)
        self.__write2ByteTxRx__(addr.ADDR_MX_CW_COMPLIANCE_MARGIN, 0)
        self.__write2ByteTxRx__(addr.ADDR_MX_CCW_COMPLIANCE_MARGIN, 0)
        self.__write1ByteTxRx__(addr.ADDR_MX_CW_COMPLIANCE_SLOPE, 32)
        self.__write1ByteTxRx__(addr.ADDR_MX_CCW_COMPLIANCE_SLOPE, 32)

        if self.check_comm_result():
            print("\nDynamixel has been successfully connected, torque mode enabled \n")

    def disable_motor(self):
        """
        Disable the motor
        """
        self.__write1ByteTxRx__(addr.ADDR_MX_TORQUE_ENABLE, addr.TORQUE_DISABLE)

        if self.check_comm_result():
            print("\nDynamixel has been successfully disconnected, torque mode disabled \n")
