package frc.team1918.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1918.robot.Constants;
import frc.team1918.robot.Dashboard;
import frc.team1918.robot.Helpers;
import frc.team1918.robot.SwerveModule;

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
	private int debug_ticks;
	private static double desiredAngle; //Used for driveStraight function
	private static boolean angleLocked = false;

	//initialize 4 swerve modules
	private static SwerveModule m_dtFL = new SwerveModule("dtFL", Constants.Swerve.FL.constants); // Front Left
	private static SwerveModule m_dtFR = new SwerveModule("dtFR", Constants.Swerve.FR.constants); // Front Right
	private static SwerveModule m_dtRL = new SwerveModule("dtRL", Constants.Swerve.RL.constants); // Rear Left
	private static SwerveModule m_dtRR = new SwerveModule("dtRR", Constants.Swerve.RR.constants); // Rear Right
	private SwerveModule[] modules = {m_dtFL, m_dtFR, m_dtRL, m_dtRR};

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
			getHeading(),
			m_dtFL.getState(),
			m_dtFR.getState(),
			m_dtRL.getState(),
			m_dtRR.getState()
		);

		updateDashboard();
		for (SwerveModule module: modules) {
			module.updateDashboard();
		}
	}

	public static void updateDashboard() {
		// Dashboard.DriveTrain.setHeading(getHeading().getDegrees());
		// Dashboard.DriveTrain.setX(getPose().getX());
		// Dashboard.DriveTrain.setY(getPose().getY());
		// Dashboard.DriveTrain.setCurrentAngle(getPose().getRotation().getRadians());
		// Dashboard.DriveTrain.setTargetAngle(m_targetPose.getRotation().getRadians());
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
     * @return the robot's heading as a Rotation2d
     */
	public Rotation2d getHeading() {
		double raw_yaw = m_gyro.getYaw();
		double calc_yaw = raw_yaw;
		if (0.0 > raw_yaw) { //yaw is negative
			calc_yaw += 360.0;
		}
		calc_yaw *= (Constants.Swerve.kGyroReversed ? -1.0 : 1.0);
		return Rotation2d.fromDegrees(-calc_yaw);
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
		double rotRPS = rot * Constants.DriveTrain.kMaxRotationRadiansPerSecond;
		ChassisSpeeds speeds = (fieldRelative) ? ChassisSpeeds.fromFieldRelativeSpeeds(fwdMPS, strMPS, rotRPS, getRot2d()) : new ChassisSpeeds(fwdMPS, strMPS, rotRPS);
		if (speeds.vxMetersPerSecond == 0 && speeds.vyMetersPerSecond == 0 && speeds.omegaRadiansPerSecond == 0) {
			brake();
			return;
		}
		if (Helpers.Debug.debugThrottleMet(debug_ticks)) {
			Helpers.Debug.debug( (fieldRelative) ? speeds.toString()+" fieldCentric" : speeds.toString()+" robotCentric");
		}
		debug_ticks++;
		var swerveModuleStates = Constants.Swerve.kDriveKinematics.toSwerveModuleStates(speeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.kMaxSpeedMetersPerSecond);
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

	//Stops all modules
	public void brake() {
		for (SwerveModule module: modules) {
			module.setDesiredState(new SwerveModuleState(0, module.getState().angle));
		}
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

	//#region GYRO STUFF
	public static AHRS getm_gyro() {
        return m_gyro;
	}

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
	public void setAllDriveBrakeMode(boolean b) {
		for (SwerveModule module: modules) {
			module.setBrakeMode("drive",b);
		}
	}

	public void setAllTurnBrakeMode(boolean b) {
		for (SwerveModule module: modules) {
			module.setBrakeMode("turn", b);
		}
	}
	//#endregion MOTOR CONTROLLER STUFF
}