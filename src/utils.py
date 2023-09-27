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
    return rad / 0.00506145378


def rot_to_rad(rot):
    """
    :param rot: value in units per rotation of motor
    :return: the angle in radians
    """
    # 0.00506145378 = 0.29 * 3.141592 / 180
    return rot * 0.00506145378
