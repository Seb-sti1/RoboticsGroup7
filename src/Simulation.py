import threading
import time
import ControlTableAddress as addr
from utils import rot_to_rad, rad_to_rot

SIM_TIME = 0.3


class SimulatedPacketHandler:
    def __init__(self, init=0):
        self.position: float = init  # rad
        self.speed: float = 0  # rad/s
        self.goal: float = init  # rad
        self.thread = None
        self.stop_thread = False
        self.last_cycle = 0
        self.dxl_id = -1

    def write1ByteTxRx(self, port_num, dxl_id, address, value):
        self.dxl_id = dxl_id
        if address == addr.ADDR_MX_TORQUE_ENABLE:
            if value == addr.TORQUE_ENABLE:
                if self.thread is None:
                    self.__start_cycle__()
            else:
                self.__stop_cycle__()
        return addr.COMM_SUCCESS, 0

    def write2ByteTxRx(self, port_num, dxl_id, address, value):
        if address == addr.ADDR_MX_GOAL_POSITION:
            self.goal = rot_to_rad(value)
        elif address == addr.ADDR_MX_MOVING_SPEED:
            self.speed = value / 1023 * 11.9380496
        return addr.COMM_SUCCESS, 0

    def read2ByteTxRx(self, port_num, dxl_id, address):
        if address == addr.ADDR_MX_PRESENT_POSITION:
            return rad_to_rot(self.position), addr.COMM_SUCCESS, 0

    def getRxPacketError(self, error):
        return ""

    def getTxRxResult(self, dxl_comm_result):
        return ""

    def __start_cycle__(self):
        self.last_cycle = time.time()

        def update_position():
            while not self.stop_thread:
                if abs(self.goal - self.position) > 0.0001:
                    if self.goal > self.position:
                        self.position = min(self.goal, self.position + self.speed * (time.time() - self.last_cycle))
                    else:
                        self.position = max(self.goal, self.position - self.speed * (time.time() - self.last_cycle))
                self.last_cycle = time.time()
                time.sleep(SIM_TIME)
        self.thread = threading.Thread(target=update_position)
        self.thread.start()

    def __stop_cycle__(self):
        self.stop_thread = True
        self.thread.join()
        self.thread = None
