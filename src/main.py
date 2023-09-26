from Robot import Robot


if __name__ == '__main__':
    our_robot = Robot([1, 2, 3, 4], [(-130, 130), (-180, 0), (-100, 100), (-100, 100)])

    our_robot.move_to([10, -10, -10, -10])

