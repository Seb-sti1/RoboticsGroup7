import dynamixel_sdk.src.dynamixel_sdk as dxl
from src.Servo import Servo

BAUDRATE = 1000000  # Baudrate for Motors
DEVICENAME = '/dev/ttyACM0'  # Check which port is being used on your controller
# ex) Windows: 'COM1'   Linux: '/dev/ttyUSB0' Mac: '/dev/tty.usbserial-*'

class Robot:
    def __init__(self, ids, angles, port=DEVICENAME):

        self.port = port

        self.port_handler = dxl.PortHandler(self.port)
        if self.port_handler.openPort():
            print("Succeeded to open the port")
        else:
            raise Exception("Failed to open the port")

        if self.port_handler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            raise Exception("Failed to change the baudrate")

        # create the servos
        self.servos = []
        for motor_id, angle in zip(ids, angles):
            self.servos.append(Servo(motor_id, angle[0], angle[1], self.port_handler))

        # enable all servos
        for servo in self.servos:
            servo.enable_motor()
            servo.set_position(0)

    def move_to(self, angles):
        """
        Move the robot to the given angles
        :param angles: a array of angles
        """
        if len(angles) != len(self.servos):
            raise Exception("The number of angles does not match the number of servos")

        for servo, angle in zip(self.servos, angles):
            servo.set_position(angle)
