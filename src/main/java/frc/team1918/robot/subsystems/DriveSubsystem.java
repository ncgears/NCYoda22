package frc.team1918.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1918.robot.Constants;
import frc.team1918.robot.Dashboard;
import frc.team1918.robot.Helpers;
import frc.team1918.robot.SwerveModule;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//kinematics and odometry
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
//import edu.wpi.first.math.geometry.Translation2d;

public class DriveSubsystem extends SubsystemBase {

	private static DriveSubsystem instance;
	private static double l = Constants.Global.ROBOT_LENGTH, w = Constants.Global.ROBOT_WIDTH, r = Math.sqrt((l * l) + (w * w));
	private int debug_ticks, dash_gyro_ticks, dash_dt_ticks;
	private static double desiredAngle; //Used for driveStraight function
	private static boolean angleLocked = false;

	//initialize 4 swerve modules
	private static SwerveModule m_dtFL = new SwerveModule("dtFL", Constants.Swerve.FL.constants); // Front Left
	private static SwerveModule m_dtFR = new SwerveModule("dtFR", Constants.Swerve.FR.constants); // Front Right
	private static SwerveModule m_dtRL = new SwerveModule("dtRL", Constants.Swerve.RL.constants); // Rear Left
	private static SwerveModule m_dtRR = new SwerveModule("dtRR", Constants.Swerve.RR.constants); // Rear Right
	//initialize gyro object
	private static AHRS m_gyro = new AHRS(SPI.Port.kMXP);
	//intialize odometry class for tracking robot pose
	SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(Constants.Swerve.kDriveKinematics, m_gyro.getRotation2d());

	public static DriveSubsystem getInstance() {
		if (instance == null)
			instance = new DriveSubsystem();
		return instance;
	}

	public DriveSubsystem() { //initialize the class
		
	}

	@Override
	public void periodic() {
		// Update the odometry in the periodic block
		m_odometry.update(
			new Rotation2d(getHeading()),
			m_dtFL.getState(),
			m_dtFR.getState(),
			m_dtRL.getState(),
			m_dtRR.getState()
		);

		if(dash_gyro_ticks % 5 == 0) {
			Dashboard.Gyro.setGyroAngle(Helpers.General.roundDouble(m_gyro.getAngle(),3)); 
		} 
		dash_gyro_ticks++;
		if(dash_dt_ticks % 5 == 0) {
			m_dtFL.updateDashboard();
		}
		dash_dt_ticks++;
	}

	/**
	 * Returns the currently-estimated pose of the robot.
	 * @return The pose.
	 */
	public Pose2d getPose() {
	  return m_odometry.getPoseMeters();
	}

	/**
     * Returns the heading of the robot.
     * @return the robot's heading in degrees, from -180 to 180
     */
	public double getHeading() {
		return m_gyro.getRotation2d().getDegrees() * (Constants.Swerve.kGyroReversed ? -1.0 : 1.0);
	}

	/**
     * Returns the heading of the robot.
     * @return the robot's heading as a Rotation2d
     */
	public Rotation2d getHeadingAsRotation2d() {
		double raw_yaw = m_gyro.getYaw();
		double calc_yaw = raw_yaw;
		if (0.0 > raw_yaw) { //yaw is negative
			calc_yaw += 360.0;
		}
		return Rotation2d.fromDegrees(-calc_yaw); //TODO: Tie this to kGyroReversed, but move it out of swerve (why is it there?)
	}

	/**
	 * Returns the turn rate of the robot.
	 * @return The turn rate of the robot, in degrees per second
	 */
	public double getTurnRate() {
		return m_gyro.getRate() * (Constants.Swerve.kGyroReversed ? -1.0 : 1.0);
	}

	/**
	 * Resets the odometry to the specified pose.
	 *
	 * @param pose The pose to which to set the odometry.
	 */
	public void resetOdometry(Pose2d pose) {
	  m_odometry.resetPosition(pose, m_gyro.getRotation2d());
	}

	public void lockAngle() {
		desiredAngle = Helpers.General.roundDouble(m_gyro.getAngle(), 3);
		angleLocked = true;
		Helpers.Debug.debug("Angle Locked to "+desiredAngle);
	}

	public void unlockAngle() {
		if(angleLocked) Helpers.Debug.debug("Angle Unlocked");
		angleLocked = false;
	}

	/**
	 * Method to drive the robot using joystick info.
	 * @param fwd Speed of the robot in the x direction (forward).
	 * @param str Speed of the robot in the y direction (sideways).
	 * @param rot Angular rate of the robot.
	 * @param fieldRelative Whether the provided x and y speeds are relative to the field.
	 */
	@SuppressWarnings("ParameterName")
	public void drive(double fwd, double str, double rot, boolean fieldRelative) {

		if(Constants.DriveTrain.useDriveStraight) {
			if(rot == 0) { //We are not applying rotation
				if(!angleLocked) { //We havent locked an angle, so lets save the desiredAngle and lock it
					lockAngle();
				} else {
					if (Math.abs(fwd) > 0 || Math.abs(str) > 0) { //Only do angle correction while moving, for safety reasons
						rot += calcAngleStraight(desiredAngle,m_gyro.getAngle(),Constants.DriveTrain.kDriveStraight_P); //Add some correction to the rotation to account for angle drive
					}
				}
			}
		}
		double fwdMPS = fwd * Constants.DriveTrain.kMaxMetersPerSecond;
		double strMPS = str * Constants.DriveTrain.kMaxMetersPerSecond;
		double rotRPS = str * Constants.DriveTrain.kMaxRotationRadiansPerSecond;
		var swerveModuleStates =
		Constants.Swerve.kDriveKinematics.toSwerveModuleStates(fieldRelative
					? ChassisSpeeds.fromFieldRelativeSpeeds(fwdMPS, strMPS, rotRPS, getRot2d())
					: new ChassisSpeeds(fwdMPS, strMPS, rotRPS));
		if (Helpers.Debug.debugThrottleMet(debug_ticks)) {
			Helpers.Debug.debug((fieldRelative)
			? ChassisSpeeds.fromFieldRelativeSpeeds(fwdMPS, strMPS, rotRPS, getRot2d()).toString()+" fieldCentric"
			: new ChassisSpeeds(fwdMPS, strMPS, rotRPS).toString()+" robotCentric");
		}
		debug_ticks++;
		SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.kMaxSpeedMetersPerSecond);
		// SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, Constants.Swerve.kMaxSpeedMetersPerSecond);
		if(!Constants.Swerve.FL.isDisabled) m_dtFL.setDesiredState(swerveModuleStates[0]);
		if(!Constants.Swerve.FR.isDisabled) m_dtFR.setDesiredState(swerveModuleStates[1]);
		if(!Constants.Swerve.RL.isDisabled) m_dtRL.setDesiredState(swerveModuleStates[2]);
		if(!Constants.Swerve.RR.isDisabled) m_dtRR.setDesiredState(swerveModuleStates[3]);
	}
	public void drive(ChassisSpeeds speeds, boolean normalize) {
		if (speeds.vxMetersPerSecond == 0 && speeds.vyMetersPerSecond == 0 && speeds.omegaRadiansPerSecond == 0) {
			brake();
			return;
		}
		SwerveModuleState[] swerveModuleStates = Constants.Swerve.kDriveKinematics.toSwerveModuleStates(speeds);
		if (normalize) SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.kMaxSpeedMetersPerSecond);
		// setModuleStates(swerveModuleStates);
		if(!Constants.Swerve.FL.isDisabled) m_dtFL.setDesiredState(swerveModuleStates[0]);
		if(!Constants.Swerve.FR.isDisabled) m_dtFR.setDesiredState(swerveModuleStates[1]);
		if(!Constants.Swerve.RL.isDisabled) m_dtRL.setDesiredState(swerveModuleStates[2]);
		if(!Constants.Swerve.RR.isDisabled) m_dtRR.setDesiredState(swerveModuleStates[3]);
	}

	public void brake() {
		m_dtFL.setDesiredState(new SwerveModuleState(0, m_dtFL.getState().angle));
		m_dtFR.setDesiredState(new SwerveModuleState(0, m_dtFR.getState().angle));
		m_dtRL.setDesiredState(new SwerveModuleState(0, m_dtRL.getState().angle));
		m_dtRR.setDesiredState(new SwerveModuleState(0, m_dtRR.getState().angle));
	}

	/**
	 * This returns and angle correction (in degrees)
	 * @param targetAngle [double] target angle (heading) of the robot in degrees
	 * @param currentAngle [double] current angle (heading) of the robot in degrees
	 * @param kP [double] proportional multiplier for straight angle correction
	 * @return [double] Degrees of correction using kP multiplier (to control how quickly we correct back to straight)
	 */
	public double calcAngleStraight(double targetAngle, double currentAngle, double kP) {
		double errorAngle = (targetAngle - currentAngle) % 360.0;
		double correction = errorAngle * kP;
		return correction;
	}

	/**
	 * Sets the swerve ModuleStates.
	 * @param desiredStates The desired SwerveModule states.
	 */
	public void setModuleStates(SwerveModuleState[] desiredStates) {
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.kMaxSpeedMetersPerSecond);
		// SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, Constants.Swerve.kMaxSpeedMetersPerSecond);
		m_dtFL.setDesiredState(desiredStates[0]);
		m_dtFR.setDesiredState(desiredStates[1]);
		m_dtRL.setDesiredState(desiredStates[2]);
		m_dtRR.setDesiredState(desiredStates[3]);
	}

	public static AHRS getm_gyro() {
        return m_gyro;
	}

	public static void setDrivePower(double flPower, double frPower, double rlPower, double rrPower) {
		if (!Constants.DriveTrain.DT_DRIVE_DISABLED) {
			m_dtFL.setDrivePower(flPower);
			m_dtFR.setDrivePower(frPower);
			m_dtRL.setDrivePower(rlPower);
			m_dtRR.setDrivePower(rrPower);
		}
	}

	public static void setTurnPower(double flPower, double frPower, double rlPower, double rrPower) {
	    m_dtFL.setTurnPower(flPower);
		m_dtFR.setTurnPower(frPower);
		m_dtRL.setTurnPower(rlPower);
		m_dtRR.setTurnPower(rrPower);
	}

	public static void setLocation(double flLoc, double frLoc, double rlLoc, double rrLoc) {
	    m_dtFL.setTurnLocation(flLoc);
		m_dtFR.setTurnLocation(frLoc);
		m_dtRL.setTurnLocation(rlLoc);
		m_dtRR.setTurnLocation(rrLoc);
	}

	public void stopAllDrive() {
	    m_dtFL.stopDrive();
		m_dtFR.stopDrive();
		m_dtRL.stopDrive();
		m_dtRR.stopDrive();
	}

	/*
	 * Drive methods
	 */
	public static void swerveDrive(double fwd, double str, double rot) {
		double a = str - (rot * (l / r));
		double b = str + (rot * (l / r));
		double c = fwd - (rot * (w / r));
		double d = fwd + (rot * (w / r));

		//Wheel Speed
		double ws1 = Math.sqrt((b * b) + (c * c)); //FR
		double ws2 = Math.sqrt((a * a) + (c * c)); //RR
		double ws3 = Math.sqrt((a * a) + (d * d)); //RL
		double ws4 = Math.sqrt((b * b) + (d * d)); //FL

		//Wheel Angle
		double wa1 = Math.atan2(b, c); //FR
		double wa2 = Math.atan2(a, c); //RR
		double wa3 = Math.atan2(a, d); //RL
		double wa4 = Math.atan2(b, d); //FL

		double max = ws1;
		max = Math.max(max, ws2);
		max = Math.max(max, ws3);
		max = Math.max(max, ws4);
		if (max > 1) {
			ws1 /= max;
			ws2 /= max;
			ws3 /= max;
			ws4 /= max;
		}
		SmartDashboard.putNumber("ws1", ws1);
		SmartDashboard.putNumber("ws2", ws2);
		SmartDashboard.putNumber("ws3", ws3);
		SmartDashboard.putNumber("ws4", ws4);
		SmartDashboard.putNumber("wa1", wa1);
		SmartDashboard.putNumber("wa2", wa2);
		SmartDashboard.putNumber("wa3", wa3);
		SmartDashboard.putNumber("wa4", wa4);

		DriveSubsystem.setDrivePower(ws4, ws1, ws3, ws2);
		DriveSubsystem.setLocation(wa4, wa1, wa3, wa2);
	}
	//#region GYRO STUFF
	public void resetGyro() {
		Helpers.Debug.debug("Gyro Reset");
		m_gyro.reset();
		resetOdometry(getPose());
	}

	public static Rotation2d getRot2d() {
		return Rotation2d.fromDegrees(getGyroAngle());
	}

	public static double getGyroAngle() {
		return m_gyro.getAngle();
	}

	public static double getGyroAngleInRad() {
		return m_gyro.getAngle() * (Math.PI / 180d);
	}
	//#endregion GYRO STUFF

	//#region MOTOR CONTROLLER STUFF
	public static void setAllDriveBrakeMode(boolean b) {
		m_dtFL.setBrakeMode("drive", b);
		m_dtFR.setBrakeMode("drive", b);
		m_dtRL.setBrakeMode("drive", b);
		m_dtRR.setBrakeMode("drive", b);
	}

	public static void setAllTurnBrakeMode(boolean b) {
		m_dtFL.setBrakeMode("turn", b);
		m_dtFR.setBrakeMode("turn", b);
		m_dtRL.setBrakeMode("turn", b);
		m_dtRR.setBrakeMode("turn", b);
	}
	//#endregion MOTOR CONTROLLER STUFF
}