package frc.team1918.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
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
	private double[] lastDistances;
	private double lastTime;
	private final Timer timer;
	private static double yawOffset = 0.0; //offset to account for different starting positions

	//initialize 4 swerve modules
	private static SwerveModule m_dtFL = new SwerveModule("dtFL", Constants.Swerve.FL.constants); // Front Left
	private static SwerveModule m_dtFR = new SwerveModule("dtFR", Constants.Swerve.FR.constants); // Front Right
	private static SwerveModule m_dtRL = new SwerveModule("dtRL", Constants.Swerve.RL.constants); // Rear Left
	private static SwerveModule m_dtRR = new SwerveModule("dtRR", Constants.Swerve.RR.constants); // Rear Right
	private SwerveModule[] modules = {m_dtFL, m_dtFR, m_dtRL, m_dtRR};

	//initialize gyro object
	private static AHRS m_gyro = new AHRS(SPI.Port.kMXP);
	//intialize odometry class for tracking robot pose
	static SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(Constants.Swerve.kDriveKinematics, m_gyro.getRotation2d());

	public static DriveSubsystem getInstance() {
		if (instance == null)
			instance = new DriveSubsystem();
		return instance;
	}

	public DriveSubsystem() { //initialize the class
		m_gyro.calibrate();
		m_odometry = new SwerveDriveOdometry(Constants.Swerve.kDriveKinematics, getHeading());
		for (SwerveModule module: modules) {
			module.resetDistance();
			module.syncTurningEncoders();
		}

		// m_targetPose = m_odometry.getPoseMeters();
		// m_thetaController.reset();
		// m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
		lastDistances = new double[]{
			m_dtFL.getDriveDistanceMeters(),
			m_dtFR.getDriveDistanceMeters(),
			m_dtRL.getDriveDistanceMeters(),
			m_dtRR.getDriveDistanceMeters()
		};
		timer = new Timer();
		timer.reset();
		timer.start();
		lastTime = 0;
	}

	@Override
	public void periodic() {
		updateOdometry();
		updateDashboard();
		for (SwerveModule module: modules) {
			// module.updateDashboard();
		}
	}

	public void updateDashboard() {
		Dashboard.DriveTrain.setHeading(getHeading().getDegrees());
		Dashboard.DriveTrain.setX(getPose().getX());
		Dashboard.DriveTrain.setY(getPose().getY());
		Dashboard.DriveTrain.setCurrentAngle(getPose().getRotation().getDegrees());
		Dashboard.DriveTrain.setDesiredAngle(desiredAngle);
		// Dashboard.DriveTrain.setTargetAngle(m_targetPose.getRotation().getRadians());
	}

		/**
     * Returns the heading of the robot.
     * @return the robot's heading as a Rotation2d
     */
	public Rotation2d getHeading() {
		double raw_yaw = m_gyro.getYaw() - (double)yawOffset;  //always subtract the offset
		double calc_yaw = raw_yaw;
		if (0.0 > raw_yaw) { //yaw is negative
			calc_yaw += 360.0;
		}
		calc_yaw *= (Constants.Swerve.kGyroReversed ? -1.0 : 1.0);
		return Rotation2d.fromDegrees(calc_yaw);
	}

	public void zeroHeading() {
		m_gyro.zeroYaw();
		yawOffset = 0;
	}

		/**
	 * Returns the currently-estimated pose of the robot.
	 * @return The pose.
	 */
	public Pose2d getPose() {
		return m_odometry.getPoseMeters();
	}

	/**
	 * Resets the odometry to the specified pose. Requires the current heading to account for starting position other than 0.
	 * 
	 * @param heading The current heading of the robot to offset the zero position
	 * @param pose The pose to which to set the odometry.
	 */
	public void resetOdometry(double heading, Pose2d pose) {
	  zeroHeading();
	  yawOffset = heading;
	  m_odometry.resetPosition(pose, Rotation2d.fromDegrees(heading));
	}

	public void updateOdometry() {
		//this is the old way
		/*
		m_odometry.update(
			getHeading(),
			m_dtFL.getState(),
			m_dtFR.getState(),
			m_dtRL.getState(),
			m_dtRR.getState()
		);
		*/
		//this is the new way
		double[] distances = new double[] {
			m_dtFL.getDriveDistanceMeters(),
			m_dtFR.getDriveDistanceMeters(),
			m_dtRL.getDriveDistanceMeters(),
			m_dtRR.getDriveDistanceMeters()
		};
		double time = timer.get();
		double dt = time - lastTime;
		lastTime = time;
		if (dt == 0) return;
		m_odometry.updateWithTime(time,
			getHeading(),
			new SwerveModuleState((distances[0] - lastDistances[0]) / dt, m_dtFL.getState().angle),
			new SwerveModuleState((distances[1] - lastDistances[1]) / dt, m_dtFR.getState().angle),
			new SwerveModuleState((distances[2] - lastDistances[2]) / dt, m_dtRL.getState().angle),
			new SwerveModuleState((distances[3] - lastDistances[3]) / dt, m_dtRR.getState().angle)
		);
		lastDistances = distances;
	}

	public void lockAngle() {
		desiredAngle = getHeading().getDegrees(); //Helpers.General.roundDouble(m_gyro.getAngle(), 3);
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
						Dashboard.DriveTrain.setCorrectionAngle(calcAngleStraight(desiredAngle,getHeading().getDegrees(),Constants.DriveTrain.kDriveStraight_P));
						rot += calcAngleStraight(desiredAngle,getHeading().getDegrees(),Constants.DriveTrain.kDriveStraight_P); //Add some correction to the rotation to account for angle drive
					}
				}
			}
		}
		double fwdMPS = fwd * Constants.DriveTrain.kMaxMetersPerSecond;
		double strMPS = str * Constants.DriveTrain.kMaxMetersPerSecond;
		double rotRPS = rot * Constants.DriveTrain.kMaxRotationRadiansPerSecond;
		//getHeading was getRot2d which used m_gyro.getAngle()
		ChassisSpeeds speeds = (fieldRelative) ? ChassisSpeeds.fromFieldRelativeSpeeds(fwdMPS, strMPS, rotRPS, getHeading()) : new ChassisSpeeds(fwdMPS, strMPS, rotRPS);
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

	/** Resets the drive encoders to read a position of 0. */
	public void resetEncoders() {
		for (SwerveModule module: modules) {
			module.resetEncoders();
		}
	}

	/** Resets the drive distances to read 0. */
	public void resetDistances() {
		for (SwerveModule module: modules) {
			module.resetDistance();
		}
	}

	/** Moves the swerve modules to their 0 position (in current loop). */
	public void homeSwerves() {
		for (SwerveModule module: modules) {
			module.homeSwerve();
		}
	}

	public boolean swervesAtHome() {
		boolean home = true;
		// boolean[] atHome;
		// int i = 0;
		for (SwerveModule module: modules) {
			// atHome[i] = module.getTurnError() <= Constants.Swerve.DEFAULT_TURN_ALLOWED_ERROR;
			home &= module.getTurnError() <= Constants.Swerve.DEFAULT_TURN_ALLOWED_ERROR;
			// i++;
		}
		// for (int j=0; j<atHome.length; j++) {
			
		// }
		return home;
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
		if (Math.abs(correction) > .5) return Math.signum(correction) * 0.01;
		return correction;
	}

	//#region GYRO STUFF
	public static AHRS getm_gyro() {
        return m_gyro;
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