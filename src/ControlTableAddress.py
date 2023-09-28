ADDR_MX_CW_COMPLIANCE_MARGIN = 26
ADDR_MX_CCW_COMPLIANCE_MARGIN = 27
ADDR_MX_CW_COMPLIANCE_SLOPE = 28
ADDR_MX_CCW_COMPLIANCE_SLOPE = 29
ADDR_MX_MOVING_SPEED = 32
ADDR_MX_PUNCH = 48

ADDR_MX_GOAL_POSITION = 30  # Control table address for reading goal position
ADDR_MX_PRESENT_POSITION = 36  # Control table address for reading current position
ADDR_MX_CW_ANGLE_LIMIT = 6  # Control table address for cw angle limit (initially 0)
ADDR_MX_CCW_ANGLE_LIMIT = 8  # Control table address for ccw angle limit (initially 1023)

ADDR_MX_TORQUE_ENABLE = 24  # Control table address for enabling torque mode
ADDR_MX_TORQUE_LIMIT = 34
TORQUE_ENABLE = 1  # Value for enabling the torque
TORQUE_DISABLE = 0  # Value for disabling the torque

DXL_MOVING_STATUS_THRESHOLD = 10  # Dynamixel moving status threshold

COMM_SUCCESS = 0  # Communication Success result value
COMM_TX_FAIL = -1001  # Communication Tx Failed

PROTOCOL_VERSION = 1.0  # See which protocol version is used in the Dynamixel
