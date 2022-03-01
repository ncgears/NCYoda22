from trajectory_generator import trajectory_generator
from drive.swerve_drive import swerve_drive
import trajectory_io
import math

def main():
    drive = swerve_drive(
        # Wheelbase x/y (length/width)
        0.66, 0.584,
        # Bumper length/width (meters)
        0.914, 0.838,
        # Mass (kg)/moi (kg/m^2)
        54.4, 5.6,
        # Max velocity (RPS)/force (Nm) at 60A, adjusted for gearing
        81, 1.07,
        # 50, 1,
        # Wheel radius (meters)
        0.071)

    generator = trajectory_generator(drive)

    # array points are [xMeters,yMeters,rotationInRadians]

    # generator.generate(
    #     [[0,2,0],
    #     [-1.0,0,-0.15]],
    #     "Test"
    # )

    generator.generate(
        [[0,0,0],
        [-1.0,0,-0.15]],
        "OneMeterForward"
    )

    generator.generate(
        [[-1.0,0,-0.15],
        [0,2,-math.pi/2],
        [-0.42,6.121,-1]],
        "CollectBall3AndBall4"
    )

    generator.generate(
        [[-0.42,6.121,-1],
        [-0.2,0.5,-0.575]],
        "GoHome"
    )

main()