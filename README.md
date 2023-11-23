# Robotics Group 7

This repository allows to control a robotic arm with 4 [AX-12A](https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/)
servos.

## Units conventions

If not specified otherwise:
- All angles are in radians.
- All distances are in meters.

## Frames

The robot is composed of 4 frames following the [Denavit-Hartenberg](https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters) convention.

The frames are defined as follows:

| Frame | theta   | d    | a     | alpha |
|-------|---------|------|-------|-------|
| 1     | theta_1 | 0.05 | 0     | pi/2  |
| 2     | theta_2 | 0    | 0.093 | 0     |
| 3     | theta_3 | 0    | 0.093 | 0     |
| 4     | theta_4 | 0    | 0.05  | 0     |

_The end effector isn't represented in the current version._

## Description of the files

- `Servo.py`: contains the class `Servo` which allows to control a servo motor.
- `RobotDescription.py`: contains the class `RobotDescription` which allows to describe the robot, especially the frames and the reverse kinematics.
- `Robot.py`: contains the class `Robot` which allows to control the robot. It uses the class `Servo`.
- `Simulation.py`: contains the class `Simulation` which allows to simulate the `PacketHandler` from the dynamixel library.
- `Visualizer.py`: contains the class `Visualizer` which allows to visualize the robot in 3D.
- `Camera.py`: contains the class `Camera` which allows to calibrate and control the camera.
- `test_reverse_kinematics.py`: contains the tests for the reverse kinematics.

## How to use

### Installation

```bash
pip install -r requirements.txt
```

_Note:_ tqdm and mpi4py are only required for the test of the reverse kinematics.

### Start

Change the value of `sim` in `main.py` to `False` to use the real robot.
The calibration of the camera requires a chessboard image.

```bash
python main.py
```

Then follow the instructions in the terminal.

### Start the test

```bash
mpiexec -n 8 python -m mpi4py test_reverse_kinematics.py
```