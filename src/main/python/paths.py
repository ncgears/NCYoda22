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
    # X is length of field, so -X is toward driver station, +X is away from driver station
    # Y is width of field, so -Y is left, +Y is right (from driver view) //TO BE VERIFIED
    # Rotation is in radians, so 2pi/180 = 6.28rad, or a full rotation
    # if we setup offset from rotational zero, we need to account for this in our auton

    # generator.generate(
    #     [[0,2,0],
    #     [-1.0,0,-0.15]],
    #     "Test"
    # )

    # generator.generate(
    #     [[0,0,0],
    #     [1.0,0,0]],
    #     "OneMeterForward"
    # )

    # generator.generate(
    #     [[0,0,0],
    #     [-1.0,0,0]],
    #     "OneMeterBackward"
    # )

    # generator.generate(
    #     [[0,0,0],
    #     [3.2,0,0]],
    #     "TwoMetersForward"
    # )

    # generator.generate(
    #     [[0,0,0],
    #     [-3.2,0,0]],
    #     "TwoMetersBackward"
    # )

    # generator.generate(
    #     [[-1.0,0,-0.15],
    #     [0,2,-math.pi/2],
    #     [-0.42,6.121,-1]],
    #     "CollectBall3AndBall4"
    # )

    # generator.generate(
    #     [[-0.42,6.121,-1],
    #     [-0.2,0.5,-0.575]],
    #     "GoHome"
    # )

    # generator.generate(
    #     [
    #         [0,0,0],
    #         [0.4,0.4,0],
    #         [1.8,0.4,0],
    #     ],
    #     "ar1BallOne"
    # )

    generator.generate(
        [
            [1.8,0.4,0],
            [2.2,0.75,-1.0],
            [2.0,4.3,-1.0],
            [1.1,2.1,1.0],
        ],
        "ar1BallTwo"
    )

main()