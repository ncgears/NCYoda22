
package frc.team1918.robot;
import frc.team1918.robot.utils.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

//Sometimes it is useful to comment out the following to see what variables or what controller buttons are not assigned yet
@SuppressWarnings("unused") //We silence the "unused variables" warnings in VSCode
/**
 * Constants let us quickly define the characteristics of our robot without having to search through code
 */
public class Constants {
    /**
     * Constants that are Global for the robot
     */
    public static final class Global {
        //Global Constants
        public final static boolean CAMERA_ENABLED = false; //set to false if UsbCamera is removed
        public final static boolean SWERVE_SENSOR_NONCONTINUOUS = false;
        public final static int kTimeoutMs = 30; //Timeout for reporting in DS if action fails, set to 0 to skip confirmation
        public final static int PID_PRIMARY = 0;  //Talon PID slot for primary loop
        public final static int ROBOT_WIDTH = 23; //Width of the robot frame (from the pivot of the wheels)
        public final static int ROBOT_LENGTH = 26; //Length of the robot frame (from the pivot of the wheels)
        public final static boolean DEBUG_ENABLED_DEFAULT = true; //Default starting state of debug mode
        public final static int DEBUG_RECURRING_TICKS = 100; //Periodic cycles for recurring debug messages
        public final static int DASH_RECURRING_TICKS = 50; //Periodic cycles for dashboard updates
        public final static boolean tuningMode = false; //Enable tunable numbers
    }

    /**
     * Constants for the Autonomous subsystem
     */
    public static final class Auton {
        public final static boolean isDisabled = true; //Disable autonomous
        public final static double kMaxSpeedMetersPerSecond = 0.25;
        public final static double kMaxAccelMetersPerSecondSquared = 0.0;
        public static final double kMaxOmega =
        (kMaxSpeedMetersPerSecond / Math.hypot(0.5461 / 2.0, 0.6477 / 2.0));
    }

    /**
     * Constants for the Pneumantics system
     * This is not a subsystem, the pneumatics are controlled directly in their respective subsystems
     */
    public static final class Air {
        public final static boolean isDisabled = false; //Disable compressor
        public final static int id_CollectorSolonoid = 0; //ID of solonoid for collector
        public final static int id_HoodSolonoid = 1; //ID of solonoid for hood control
        public final static int id_ClimbHook1Solonoid = 2; //ID of solonoid for climber hook 1
        public final static int id_ClimbHook2Solonoid = 3; //ID of solonoid for climber hook 2
        public final static int id_WhirlyGigSolonoid = 4; //ID of solonoid for whirlygig
        public final static boolean stateCollectorUp = true; //State of the solenoid when COLL1 is up
        public final static boolean stateHoodUp = true; //State of the solonoid when HOOD is up
        public final static boolean stateClimbHook1Locked = true; //State of the solonoid when the hook is locked
        public final static boolean stateClimbHook2Locked = true; //State of the solonoid when the hook is locked
        public final static boolean stateWhirlygigUp = true; //State of the solenoid when CLIMBER is up
    }
 
    /**
     * Constants for the Collector subsystem
     */
    public static final class Collector {
        public final static boolean isDisabled = false; //Disable the collector subsystem
        public final static int id_Motor1 = 15; 
        public final static int id_ColorSensor1 = 4;
        public final static double kDefaultCollectorSpeed = 1.0;
    }
   
    /**
     * Constants for the Feeder subsystem
     */
    public static final class Feeder {
        public final static boolean isDisabled = false; //Disable the feeder subsystem
        public final static int id_BeamBreak1 = 0; //ID of the Beam Break 1 DIO
        public final static int id_Motor1 = 14; //ID of the Feeder Motor 1 Controller
        public final static boolean isInverted_Motor1 = false; //Invert motor direction
        public final static double speed_Motor1 = 0.5; //Feeder Motor 1 Speed
    }

    /**
     * Constants for the Shooter subsystem
     */
    public static final class Shooter {
        public final static boolean isDisabled = false; //Disable the shooter subsystem
        public final static int id_Motor1 = 16; //ID of the Shooter Motor 1 Controller
        public final static int kEncoderFullRotation = 2048; //Falcon integrated encoder is 2048
        public final static boolean isInverted_Motor1 = false; //Invert motor direction
        public final static int kMaxShooterSpeed = 5000; //Max RPM of the Shooter Motor
        public final static int kMinShooterSpeed = 100; //Min RPM of the Shooter Motor
        public final static double kSpeedIncrementSize = 25; //RPMs to change the shooter speed per increment
        public final static double kP = 0.001;
        public final static double kI = 0.0;
        public final static double kD = 0.0;
        public final static double kIZone = 0.0;
        public final static double kArbitraryFeedForward = 1/kMaxShooterSpeed;

        public static final class Positions {
            public final static int speed_Pos1 = 2400;
            public final static boolean hood_Pos1 = !Air.stateHoodUp;
            public final static String name_Pos1 = "Over The Rainbow";
        }
    }

    /**
     * Constants for the Climber subsystem
     */
    public static final class Climber {
        public static final boolean isDisabled = false; //Disable the climber subsystem
        public static final int id_Motor1 = 18; 
        public static final int id_LimitHook1 = 1;
        public static final int id_LimitHook2 = 2;
        public static final int id_PressureSensor = 3; 
    }

    /**
     * Constants for the Swerve Modules
     */
    public static final class Swerve {
        public static final boolean USE_OPTIMIZATION = true; //false to disable shortest path optimization
        public static final boolean USE_DRIVE_PID = false; //true to enable PID based drive control
        public static final boolean DISABLE_FL = false; //Disable FL Module
        public static final boolean DISABLE_FR = false; //Disable FR Module
        public static final boolean DISABLE_RL = false; //Disable RL Module
        public static final boolean DISABLE_RR = false; //Disable RR Module
        // turn pid defaults (used in module definitions)
        public static final double DEFAULT_TURN_P = 2.8; //PID P
        public static final double DEFAULT_TURN_I = 0.0; //PID I
        public static final double DEFAULT_TURN_D = 0.0; //PID D
        public static final int DEFAULT_TURN_IZONE = 0; //PID IZone
        public static final int DEFAULT_TURN_ALLOWED_ERROR = 2; //PID Allowed Error
        // swerve control definitions
        public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
        public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;
        public static final double kMaxSpeedMetersPerSecond = 3.770; //12.0fps calculated; 13.7fps per Mike
        public static final boolean kGyroReversed = false;
        // Drive Motor Characterization
        // See {@link https://first.wpi.edu/FRC/roborio/release/docs/java/edu/wpi/first/wpilibj/controller/SimpleMotorFeedforward.html}
        // How do we determine these numbers? Need to find out. These falcon numbers are from Team364 example
        public static final double driveKS = (0.667 / 12); //Static Gain //divide by 12 to convert from volts to percent output for CTRE
        public static final double driveKV = (2.44 / 12); //Velocity Gain
        public static final double driveKA = (0.27 / 12); //Acceleration Gain
        // Swerve current limiting //TODO: Needs tuning, this was borrowed from Team364 example
        // See {@link https://github.com/Team364/BaseFalconSwerve/blob/main/src/main/java/frc/robot/CTREConfigs.java}
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;
        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        //Forward Positive, Left Positive, Up Positive (NWU Convention)
        public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(Units.inchesToMeters(Global.ROBOT_LENGTH / 2), Units.inchesToMeters(-Global.ROBOT_WIDTH / 2)),
            new Translation2d(Units.inchesToMeters(Global.ROBOT_LENGTH / 2), Units.inchesToMeters(Global.ROBOT_WIDTH / 2)),
            new Translation2d(Units.inchesToMeters(-Global.ROBOT_LENGTH / 2), Units.inchesToMeters(-Global.ROBOT_WIDTH / 2)),
            new Translation2d(Units.inchesToMeters(-Global.ROBOT_LENGTH / 2), Units.inchesToMeters(Global.ROBOT_WIDTH / 2))
        );
        /**
         * Constants for Front Left Swerve Module
         */
        public static final class FL {  //TODO: PID Tuning
            public static final int DRIVE_MC_ID = 33; //Falcon500 Motor Controller ID
            public static final double DRIVE_wheelDiamOffsetMM = 0.0; //offset to wheel diam to account for wear, in mm from nominal (negative for worn wheels)
            public static final boolean DRIVE_isInverted = false;
            public static final int TURN_MC_ID = 7; //TalonSRX Motor Controller ID
            public static final boolean TURN_sensorPhase = false; //When forward/reverse of controller doesn't match forward/reverse of sensor, set to true
            public static final boolean TURN_isInverted = true; //Once sensor phase is correct, we can invert these so fwd always is green, reverse is always is red
            public static final double TURN_kP = Swerve.DEFAULT_TURN_P; //PID P (only change to override default)
            public static final double TURN_kI = Swerve.DEFAULT_TURN_I; //PID I (only change to override default)
            public static final double TURN_kD = Swerve.DEFAULT_TURN_D; //PID D (only change to override default)
            public static final int TURN_kIZone = Swerve.DEFAULT_TURN_IZONE; //PID IZONE (only change to override default)
            public static final int TURN_ALLOWED_ERROR = Swerve.DEFAULT_TURN_ALLOWED_ERROR; //PID Allowed error  (only change to override default)
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_MC_ID, DRIVE_isInverted, TURN_MC_ID, TURN_sensorPhase, TURN_isInverted, TURN_ALLOWED_ERROR, TURN_kP, TURN_kI, TURN_kD, TURN_kIZone);
        }
        /**
         * Constants for Front Right Swerve Module
         */
        public static final class FR {
            public static final int DRIVE_MC_ID = 31; //Falcon500 Motor Controller ID
            public static final double DRIVE_wheelDiamOffsetMM = 0.0; //offset to wheel diam to account for wear, in mm from nominal (negative for worn wheels)
            public static final boolean DRIVE_isInverted = false;
            public static final int TURN_MC_ID = 4; //TalonSRX Motor Controller ID
            public static final boolean TURN_sensorPhase = false; //When forward/reverse of controller doesn't match forward/reverse of sensor, set to true
            public static final boolean TURN_isInverted = true; //Once sensor phase is correct, we can invert these so fwd always is green, reverse is always is red
            public static final double TURN_kP = Swerve.DEFAULT_TURN_P; //PID P (only change to override default)
            public static final double TURN_kI = Swerve.DEFAULT_TURN_I; //PID I (only change to override default)
            public static final double TURN_kD = Swerve.DEFAULT_TURN_D; //PID D (only change to override default)
            public static final int TURN_kIZone = Swerve.DEFAULT_TURN_IZONE; //PID IZONE (only change to override default)
            public static final int TURN_ALLOWED_ERROR = Swerve.DEFAULT_TURN_ALLOWED_ERROR; //PID Allowed error  (only change to override default)
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_MC_ID, DRIVE_isInverted, TURN_MC_ID, TURN_sensorPhase, TURN_isInverted, TURN_ALLOWED_ERROR, TURN_kP, TURN_kI, TURN_kD, TURN_kIZone);
        }
        /**
         * Constants for Rear Left Swerve Module
         */
        public static final class RL {
            public static final int DRIVE_MC_ID = 35; //Falcon500 Motor Controller ID
            public static final double DRIVE_wheelDiamOffsetMM = 0.0; //offset to wheel diam to account for wear, in mm from nominal (negative for worn wheels)
            public static final boolean DRIVE_isInverted = false;
            public static final int TURN_MC_ID = 8; //TalonSRX Motor Controller ID
            public static final boolean TURN_sensorPhase = false; //When forward/reverse of controller doesn't match forward/reverse of sensor, set to true
            public static final boolean TURN_isInverted = true; //Once sensor phase is correct, we can invert these so fwd always is green, reverse is always is red
            public static final double TURN_kP = Swerve.DEFAULT_TURN_P; //PID P (only change to override default)
            public static final double TURN_kI = Swerve.DEFAULT_TURN_I; //PID I (only change to override default)
            public static final double TURN_kD = Swerve.DEFAULT_TURN_D; //PID D (only change to override default)
            public static final int TURN_kIZone = Swerve.DEFAULT_TURN_IZONE; //PID IZONE (only change to override default)
            public static final int TURN_ALLOWED_ERROR = Swerve.DEFAULT_TURN_ALLOWED_ERROR; //PID Allowed error  (only change to override default)
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_MC_ID, DRIVE_isInverted, TURN_MC_ID, TURN_sensorPhase, TURN_isInverted, TURN_ALLOWED_ERROR, TURN_kP, TURN_kI, TURN_kD, TURN_kIZone);
        }
        /**
         * Constants for Rear Right Swerve Module
         */
        public static final class RR { //Rear Right
            public static final int DRIVE_MC_ID = 32; //Falcon500 Motor Controller ID
            public static final double DRIVE_wheelDiamOffsetMM = 0.0; //offset to wheel diam to account for wear, in mm from nominal (negative for worn wheels)
            public static final boolean DRIVE_isInverted = false;
            public static final int TURN_MC_ID = 11; //TalonSRX Motor Controller ID
            public static final boolean TURN_sensorPhase = false; //When forward/reverse of controller doesn't match forward/reverse of sensor, set to true
            public static final boolean TURN_isInverted = true; //Once sensor phase is correct, we can invert these so fwd always is green, reverse is always is red
            public static final double TURN_kP = Swerve.DEFAULT_TURN_P; //PID P (only change to override default)
            public static final double TURN_kI = Swerve.DEFAULT_TURN_I; //PID I (only change to override default)
            public static final double TURN_kD = Swerve.DEFAULT_TURN_D; //PID D (only change to override default)
            public static final int TURN_kIZone = Swerve.DEFAULT_TURN_IZONE; //PID IZONE (only change to override default)
            public static final int TURN_ALLOWED_ERROR = Swerve.DEFAULT_TURN_ALLOWED_ERROR; //PID Allowed error  (only change to override default)
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_MC_ID, DRIVE_isInverted, TURN_MC_ID, TURN_sensorPhase, TURN_isInverted, TURN_ALLOWED_ERROR, TURN_kP, TURN_kI, TURN_kD, TURN_kIZone);
        }
    }

    /**
     * Constants for the DriveTrain subsystem
     */
    public static final class DriveTrain {
        ////Global Tuning
        public final static boolean DT_USE_DRIVESTRAIGHT = true; //enable driveStraight functionality in drive() method
        public final static double DT_DRIVESTRAIGHT_P = 0.065; //kP for driveStraight correction
        public static final double DT_kMaxMetersPerSecond = 3.677; //limit full stick speed meters to 12.0fps
        ////Turn Tuning
        public final static double DT_TURN_MULT_STATIONARY = 1.0; //Turn speed multiplier while not moving
        public final static double DT_TURN_MULT_MOVING = 1.0; //Turn speed multiplier while moving
        public final static boolean DT_TURN_MULT_BEFORE_DB = true; //Apply turn multiplier before deadband
        public final static int DT_TURN_ENCODER_FULL_ROTATION = 1024;
        public final static boolean DT_USE_FIELD_CENTRIC = true; //Set to true to use field-centric drive
        ////Drive Tuning
        public final static double DT_FWD_MULT = 1.0; //Fwd throttle multiplier
        public final static double DT_STR_MULT = 1.0; //Str throttle multiplier
        public final static boolean DT_DRIVE_DISABLED = false; //Set to true to disable the drive motors (for lab)
        public final static double DT_WHEEL_DIAM_MM = 77.1; //diameter of drive wheels in millimeters
        public final static int DT_DRIVE_ENCODER_FULL_ROTATION = 2048; //falcon integrated encoder is 2048
        //Falcon500 = 6380RPM  free speed : 945RPM Calculated
        public final static int DT_DRIVE_FIRST_GEARONE = 16; //swerve drive first gear set input teeth
        public final static int DT_DRIVE_FIRST_GEARTWO = 36; //swerve drive first gear set output teeth
        public final static int DT_DRIVE_SECOND_GEARONE = 15; //swerve drive second gear set input teeth
        public final static int DT_DRIVE_SECOND_GEARTWO = 45; //swerve drive second gear set output teeth
        public final static double DT_DRIVE_CONVERSION_FACTOR = 0.148148; //first_gearone / first_geartwo * second_gearone / second_geartwo
        // public final static double DT_DRIVE_CONVERSION_FACTOR = (DT_DRIVE_FIRST_GEARONE / DT_DRIVE_FIRST_GEARTWO) * (DT_DRIVE_SECOND_GEARONE / DT_DRIVE_SECOND_GEARTWO); //Conversion factor to correct RPM from SparkMax getVelocity()
    }
    
    /**
     * Constants for the Operator Interface
     * The OI is based on 2 Logitech Controllers, a driver and an operator, setup for swerve drive.
     * The driver left stick controls the forward rate (up/down), and strafe rate (left/right).
     * The driver right stick controls the rotation rate (left/right).
     */
    public static final class OI {
        public final static int OI_JOY_DRIVER = 0; //ID of Driver Joystick
        public final static int OI_JOY_OPER = 1; //ID of Operator Joystick
        public final static double OI_JOY_MIN_DEADBAND = 0.1; //Deadband for analog joystick axis minimum
        public final static double OI_JOY_MAX_DEADBAND = 0.9; //Deadband for analog joystick axis minimum

        /**
         * Constants for the Driver controller
         */
        public static final class Driver {
            public final static int AXIS_STRAFE = Logitech.AXIS_LH; //Axis that moves the robot side to side on the field
            public final static int AXIS_FWD = Logitech.AXIS_LV; //Axis that moves the robot up and down the field
            public final static int AXIS_TURN = Logitech.AXIS_RH; //Axis that controls the rotation of the robot
            public final static int BTN_LOCKANGLE = Logitech.BTN_B; //Move collector to Up position
            public final static int BTN_UNLOCKANGLE = Logitech.BTN_A; //Engage anti-backdrive for climber
            public final static int BTN_FEED_FWD = Logitech.BTN_X; //Run the mixer in the forward direction
            public final static int BTN_FEED_REV = Logitech.BTN_LB; //Reverse the mixer direction to unstick power cells
            public final static int BTN_HOMESWERVE = Logitech.BTN_Y; //Home the swerve modules
            public final static int BTN_MECHZERO = Logitech.BTN_BACK; //DRIVER MECHZERO and OPER MECHZERO are required for this
            public final static int BTN_TOG_DEBUG = Logitech.BTN_START; //Toggle the debugging console messages
            public final static int DPAD_GYRO_RESET = Logitech.DPAD_LEFT;
            //Driver controller DPAD used as range selector for shooter speed (top half and bottom half)
            public final static int DPAD_THROTUP_UL = Logitech.DPAD_UPLEFT;
            public final static int DPAD_THROTUP_UP = Logitech.DPAD_UP;
            public final static int DPAD_THROTUP_UR = Logitech.DPAD_UPRIGHT;
            public final static int DPAD_THROTDN_DL = Logitech.DPAD_DNLEFT;
            public final static int DPAD_THROTDN_DN = Logitech.DPAD_DN;
            public final static int DPAD_THROTDN_DR = Logitech.DPAD_DNRIGHT;
        }
        /**
         * Constants for the Operator controller
         */
        public static final class Operator {
            public final static int AXIS_CLIMB = Logitech.AXIS_LV; //Axis that controls the climber up and down
            public final static int AXIS_COLLECTOR_OUT = Logitech.AXIS_RT; //Axis that runs the collector out (actually a trigger button)
            public final static int BTN_SHOOT_WALL = Logitech.BTN_A; //Shoot from at the wall
            public final static int BTN_SHOOT_SHORT = Logitech.BTN_B; //Shoot from close to the wall
            public final static int BTN_SHOOT_LINE = Logitech.BTN_X; //Shoot from the initiation line
            public final static int BTN_SHOOT_TRENCH = Logitech.BTN_Y; //Shoot from the trench
            public final static int BTN_TOG_MIDDOWN = Logitech.BTN_LB; //Toggle collector arm between middle and down position
            public final static int BTN_COLLECTOR_IN = Logitech.BTN_RB; //Run the collector in
            public final static int BTN_MECHZERO = Logitech.BTN_BACK; //DRIVER MECHZERO and OPER MECHZERO are required for this
            public final static int DPAD_COLLECTOR_UP = Logitech.DPAD_UP; //Move collector to up
            public final static int DPAD_COLLECTOR_MID = Logitech.DPAD_RIGHT; //Move collector to middle
            public final static int DPAD_COLLECTOR_DOWN = Logitech.DPAD_DN; //Move collector down
        }
        /**
         * This class defines the hardware button and axis IDs for a Logitech Controller.
         * The buttons array is 1-based, but the axis array is 0-based
         */
        public static final class Logitech {
            //DO NOT EDIT THESE
            private final static int BTN_A = 1; //A Button
            private final static int BTN_B = 2; //B Button
            private final static int BTN_X = 3; //X Button
            private final static int BTN_Y = 4; //Y Button
            private final static int BTN_LB = 5; //Left Bumper (L1)
            private final static int BTN_RB = 6; //Right Bumper (R1)
            private final static int BTN_BACK = 7; //Back Button (Select)
            private final static int BTN_START = 8; //Start Button
            private final static int BTN_L = 9; //Left Stick Press (L3)
            private final static int BTN_R = 10; //Right Stick Press (R3)
            private final static int AXIS_LH = 0; //Left Analog Stick horizontal
            private final static int AXIS_LV = 1; //Left Analog Stick vertical
            private final static int AXIS_LT = 2; //Analog Left Trigger
            private final static int AXIS_RT = 3; //Analog Right Trigger
            private final static int AXIS_RH = 4; //Right Analog Stick horizontal
            private final static int AXIS_RV = 5; //Right Analog Stick vertical
            private final static int DPAD_UP = 0;
            private final static int DPAD_UPRIGHT = 45;
            private final static int DPAD_RIGHT = 90;
            private final static int DPAD_DNRIGHT = 135;
            private final static int DPAD_DN = 180;
            private final static int DPAD_DNLEFT = 225;
            private final static int DPAD_LEFT = 270;
            private final static int DPAD_UPLEFT = 315;
            private final static int DPAD_IDLE = -1; 
        }
    }
}

