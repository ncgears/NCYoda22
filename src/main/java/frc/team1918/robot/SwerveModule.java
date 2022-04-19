
package frc.team1918.robot;
//1918
import frc.team1918.robot.utils.SwerveModuleConstants;

//Talon SRX/Talon FX
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

//WPILib
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class SwerveModule {
    private WPI_TalonSRX turn;
    private WPI_TalonFX drive;
    private final double FULL_ROTATION = Constants.DriveTrain.DT_TURN_ENCODER_FULL_ROTATION;
    private final double TURN_P, TURN_I, TURN_D;
    private final int TURN_IZONE;
    private final int TURN_ALLOWED_ERROR;
    private boolean isDrivePowerInverted = false;
    private String moduleName;
    private double driveWheelDiam = Constants.Swerve.DEFAULT_WHEEL_DIAM_MM;
    private int debug_ticks1;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

 	/**
	 * 1918 Swerve Module v2022.1 - This swerve module uses a Falcon 500 (TalonFX) for drive and Talon SRX for turn (bag motor with gearbox).
     * The module uses a Lamprey Absolute encoder for positioning data
	 * @param name This is the name of this swerve module (ie. "dtFL")
     * @param moduleConstants This is a SwerveModuleConstants object containing the data for this module
	 */
    public SwerveModule(String name, SwerveModuleConstants moduleConstants){
        moduleName = name;
        drive = new WPI_TalonFX(moduleConstants.idDriveMotor, Constants.Swerve.canBus);
        turn = new WPI_TalonSRX(moduleConstants.idTurnMotor);
        isDrivePowerInverted = false;
        TURN_P = moduleConstants.turnP;
        TURN_I = moduleConstants.turnI;
        TURN_D = moduleConstants.turnD;
        TURN_IZONE = moduleConstants.turnIZone;
        TURN_ALLOWED_ERROR = moduleConstants.turnMaxAllowedError;
        driveWheelDiam = moduleConstants.driveWheelDiam;

        turn.configFactoryDefault(); //Reset controller to factory defaults to avoid wierd stuff from carrying over
        turn.set(ControlMode.PercentOutput, 0); //Set controller to disabled
        turn.setNeutralMode(NeutralMode.Brake); //Set controller to brake mode
        turn.configSelectedFeedbackSensor(  FeedbackDevice.Analog, //  FeedbackDevice.CTRE_MagEncoder_Absolute, // Local Feedback Source
                                            Constants.Global.PID_PRIMARY,				// PID Slot for Source [0, 1]
                                            Constants.Global.kTimeoutMs);				// Configuration Timeout
        turn.configFeedbackNotContinuous(Constants.Global.SWERVE_SENSOR_NONCONTINUOUS, 0); //Disable continuous feedback tracking (so 0 and 1024 are effectively one and the same)

/*  CTRE SRX Mag Encoder Setup
        turn.configSelectedFeedbackSensor ( FeedbackDevice.CTRE_MagEncoder_Relative,
                                            Constants.Global.PID_PRIMARY,
                                            Constants.Global.kTimeoutMs);
        turn.configSelectedFeedbackSensor ( FeedbackDevice.CTRE_MagEncoder_Absolute,
                                            Constants.Global.PID_AUXILLARY,
                                            Constants.Global.kTimeoutMs);
*/  

        // turn.setSelectedSensorPosition(0); //reset the talon encoder counter to 0 so we dont carry over a large error from a previous testing
        // turn.set(ControlMode.Position, 1024); //set this to some fixed value for testing
        turn.setSensorPhase(moduleConstants.turnSensorPhase); //set the sensor phase based on the constants setting for this module
        turn.setInverted(moduleConstants.turnIsInverted); //set the motor direction based on the constants setting for this module
        turn.config_kP(0, TURN_P); //set the kP for PID Tuning
        turn.config_kI(0, TURN_I);
        turn.config_kD(0, TURN_D);
        turn.config_IntegralZone(0, TURN_IZONE);
        turn.overrideLimitSwitchesEnable(false);
        turn.configAllowableClosedloopError(0, TURN_ALLOWED_ERROR); 
        if(Constants.Swerve.homeOnInit) turn.set(ControlMode.Position, getZeroPositionTicks());
        // SupplyCurrentLimitConfiguration(enabled,peak,trigger threshold current,trigger threshold time(s))
        turn.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(
            Constants.Swerve.isTurnCurrentLimitEnabled,
            Constants.Swerve.kTurnCurrentLimitAmps,
            Constants.Swerve.kTurnCurrentThresholdAmps,
            Constants.Swerve.kTurnCurrentThresholdSecs));

        drive.configFactoryDefault();
        drive.set(ControlMode.PercentOutput, 0);
        drive.setNeutralMode(NeutralMode.Brake);
        drive.setInverted(moduleConstants.driveIsInverted);
        drive.config_kP(0, 0.0005);
        drive.config_kI(0, 0.0);
        drive.config_kD(0, 0.00005);
        drive.config_IntegralZone(0, 4740);
        // SupplyCurrentLimitConfiguration(enabled,peak,trigger threshold current,trigger threshold time(s))
        // drive.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true,60.0,45.0,1.0));
        drive.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(
            Constants.Swerve.isDriveCurrentLimitEnabled,
            Constants.Swerve.kDriveCurrentLimitAmps,
            Constants.Swerve.kDriveCurrentThresholdAmps,
            Constants.Swerve.kDriveCurrentThresholdSecs));
            
        // m_drive_pidController = drive.getPIDController();
        // m_drive_pidController.setP(0.0005); //PID P
        // m_drive_pidController.setI(0.0); //PID I
        // m_drive_pidController.setD(0.00005); //PID D
        // // m_drive_pidController.setIZone(0); //IZone
        // m_drive_pidController.setFF(1/4740); //Feed forward
        // m_drive_pidController.setOutputRange(-1, 1);
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        //sensor velocity is NOT RPM, but in ticks per 100ms!
        //double rawRpm = drive.getSelectedSensorVelocity();  //getEncoder().getVelocity();
        double rawRpm = Helpers.General.ticksPer100msToRPS(drive.getSelectedSensorVelocity(), Constants.DriveTrain.DT_DRIVE_ENCODER_FULL_ROTATION, 1);
        double wheelRpm = Helpers.General.gearCalcDouble(rawRpm,Constants.DriveTrain.DT_DRIVE_FIRST_GEARONE,
            Constants.DriveTrain.DT_DRIVE_FIRST_GEARTWO,
            Constants.DriveTrain.DT_DRIVE_SECOND_GEARONE,
            Constants.DriveTrain.DT_DRIVE_SECOND_GEARTWO);
            return new SwerveModuleState(Helpers.General.rpmToMetersPerSecond(wheelRpm, this.driveWheelDiam), getTurnPositionAsRotation2d());
    }

    /**
     * Minimize the change in heading the desired swerve module state would require by potentially
     * reversing the direction the wheel spins.  If the desired destination angle is more than 90 degrees either direction from the current position, 
     * instead choose a new destination that is 180 degrees from the desired destination, but invert the drive motor speed.
     * @param desiredState The desired state.
     * @return Swerve Module State reflecting the shortest path to the desired turn and the correct drive speed
     */
    public SwerveModuleState optimize(SwerveModuleState desiredState) { 
        Rotation2d currentAngle = getTurnPositionAsRotation2d();
        double delta = deltaAdjustedAngle(desiredState.angle.getDegrees(), currentAngle.getDegrees());
        double driveOutput = desiredState.speedMetersPerSecond;
        if (Math.abs(delta) > 90) { //if the requested delta is greater than 90 degrees, invert drive speed and use 180 degrees from desired angle
            driveOutput *= -1;
            delta -= Math.signum(delta) * 180;
        }
        Rotation2d adjustedAngle = Rotation2d.fromDegrees(delta + currentAngle.getDegrees());
        return new SwerveModuleState(driveOutput, adjustedAngle);
        
        // Rotation2d delta = desiredState.angle.minus(currentAngle);
        // if (Math.abs(delta.getDegrees()) > 90.0 && Math.abs(delta.getDegrees()) < 270) { //new requested delta is between 90 and -90 (270) degrees, invert drive speed and rotate 180 degrees from desired
        //     return new SwerveModuleState(-desiredState.speedMetersPerSecond, desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0)));
        // } else { //no optimization necessary
        //     return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
        // }
    }

    /**
     * This function takes a desiredState and instructs the motor controllers to move based on the desired state
     * @param desiredState Desired state with speed and angle.
     * FL = 5570, FR = 5200, RL = 5200, RR = 4740
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState state = (Constants.Swerve.USE_OPTIMIZATION) ? optimize(desiredState) : desiredState;
        if (Constants.Swerve.USE_DRIVE_PID) {
            // TODO: Send speed to closed loop control rather than percent output. Enable USE_DRIVE_PID and test.
            double motorRpm = (Helpers.General.metersPerSecondToRPM(state.speedMetersPerSecond, this.driveWheelDiam) / Constants.DriveTrain.DT_DRIVE_CONVERSION_FACTOR);
            // Helpers.Debug.debug(moduleName+" desired mps: "+state.speedMetersPerSecond+" motorRpm: "+motorRpm);
            drive.set(ControlMode.Velocity, Helpers.General.rpsToTicksPer100ms(motorRpm/60, Constants.DriveTrain.DT_DRIVE_ENCODER_FULL_ROTATION, 1), DemandType.ArbitraryFeedForward, feedforward.calculate(state.speedMetersPerSecond));
        } else {
            double percentOutput = state.speedMetersPerSecond / Constants.Swerve.kMaxSpeedMetersPerSecond;
            drive.set(ControlMode.PercentOutput, percentOutput);
        }

        int turn_ticks = Helpers.General.radiansToTicks(state.angle.getRadians() + Constants.Swerve.kHomeOffsetRadians);
        turn.set(ControlMode.Position, turn_ticks); //TODO: Double-check this and test

        //Display output for debugging
        if(Helpers.Debug.debugThrottleMet(debug_ticks1) && state.speedMetersPerSecond != 0.0) {
            Helpers.Debug.debug(moduleName+" Speed (metersPerSecond)="+Helpers.General.roundDouble(state.speedMetersPerSecond,3)+" Turn Setpoint="+turn_ticks);
        }
        debug_ticks1++;

        // double wheelDiam = Constants.DriveTrain.DT_WHEEL_DIAM_MM - this.wheelOffsetMM;
        // SwerveModuleState state = (Constants.Swerve.USE_OPTIMIZATION) ? optimize(desiredState) : desiredState;
        // if (Constants.Swerve.USE_DRIVE_PID) {
        //     // TODO: Send speed to closed loop control rather than percent output. Enable USE_DRIVE_PID and test.
        //     double motorRpm = (Helpers.General.metersPerSecondToRPM(state.speedMetersPerSecond, wheelDiam) / Constants.DriveTrain.DT_DRIVE_CONVERSION_FACTOR);
        //     // Helpers.Debug.debug(moduleName+" desired mps: "+state.speedMetersPerSecond+" motorRpm: "+motorRpm);
        //     drive.set(ControlMode.Velocity, motorRpm, DemandType.ArbitraryFeedForward, feedforward.calculate(state.speedMetersPerSecond));
        // } else {
        //     double percentOutput = state.speedMetersPerSecond / Constants.Swerve.kMaxSpeedMetersPerSecond;
        //     drive.set(ControlMode.PercentOutput, percentOutput);
        // }
        
        // //Determine which direction we should turn to get to the desired setpoint
        // int cur_ticks = getTurnPosition();
        // int min_ticks = minTurnTicks(Helpers.General.radiansToTicks(state.angle.getRadians()), cur_ticks);
        // int turn_ticks = min_ticks + cur_ticks;
        // if(Helpers.Debug.debugThrottleMet(debug_ticks1)) {
        //     Helpers.Debug.debug(moduleName+" Speed (metersPerSecond)="+Helpers.General.roundDouble(state.speedMetersPerSecond,3)+" Turn Setpoint="+turn_ticks);
        // }
        // debug_ticks1++;

        // turn.set(ControlMode.Position, turn_ticks); //Set the turn setpoint
    }

    /**
     * This function calculates the minimum turn (in encoder ticks) based on the desired location and the current location.
     * It uses the encoder wrap-around to determine if we should turn negative past 0
     * @param desiredPosition encoder count of desired position
     * @param currentPosition encoder count of current position
     * @return An encoder count between -1024..0..1024 of the new target
     */
    public int minTurnTicks(int desiredPosition, int currentPosition) {
        int wrap = (int) Constants.DriveTrain.DT_TURN_ENCODER_FULL_ROTATION; //This is where the encoder changes from high to 0
        return Helpers.General.minChange(desiredPosition,currentPosition,wrap);
        //minChange calculates the shortest distance to the desired target, even if that means wrapping over 0.
        //Examples based on 4096 (SRX Mag Encoder)
        //Example: curTicks = 1023 (45deg), desiredTicks = 3073 (1 tick above 270deg)
        //Example result: -2047 ticks (-180deg) instead of +2049 ticks
    }

    /**
     * This function sets the turn power by percentage
	 * @param power turn power from -1.0 to 1.0
    */
    public void setTurnPower(double power){
        turn.set(ControlMode.PercentOutput, power);
    }

    /**
     * This function sets the drive power by percentage
	 * @param power drive motor power from -1.0 to 1.0
    */
    public void setDrivePower(double power){
        if (this.isDrivePowerInverted) {
            drive.set(ControlMode.PercentOutput,-power);
        } else {
            drive.set(ControlMode.PercentOutput,power);
        }
    }

    // /**
    //  * Returns the raw units of the current location of the turn sensor
    //  * @return (integer) Raw units (typically encoder ticks) of turn location
    //  */
    // public int getTurnPosition(){
    //     //TODO: Change to non-normalized?
    //     // return ((int) turn.getSelectedSensorPosition(0) & 0x3FF); //normalize to a single rotation
    //     return ((int) turn.getSelectedSensorPosition(0)); //do not normalize
    //     //return (turn.getSensorCollection().getAnalogIn()); //This gets an ADC value for the analog sensor
    //     //We may want to use a bitwise "AND" with 0x3FF (1023), 0x7FF (2047), 0xFFF (4095) to get just the most significant bits that we are interested in.
    //     //Explanation: & is a bitwise "AND" operator, and 0xFFF is 4095 in Hex, consider "0101010101 AND 1111 = 0101"
    // }

    public int getZeroPositionTicks() {
        int total = (int) turn.getSelectedSensorPosition(0);
        int curr = (int) total & Constants.DriveTrain.DT_TURN_ENCODER_FULL_ROTATION;//0x3ff; //ticks in current rotation
        return total - curr;
    }

    public void homeSwerve() {
        turn.set(ControlMode.Position, 0);
        // turn.set(ControlMode.Position, getZeroPositionTicks());
    }

    /**
     * Returns a rotation2d object representing the current location of the turn sensor
     * @return Rotation2d object of the current position
     */
    public Rotation2d getTurnPositionAsRotation2d(){
        int pos = (int) turn.getSelectedSensorPosition(0); //not normalized
        // int pos = getTurnPosition();
        return new Rotation2d(Helpers.General.ticksToRadians(pos));
    }

    /**
     * Gets the closed-loop error. The units depend on which control mode is in use. If closed-loop is seeking a target sensor position, closed-loop error
     * is the difference between target and current sensor value (in sensor units. Example 4096 units per rotation for CTRE Mag Encoder). 
     * If closed-loop is seeking a target sensor velocity, closed-loop error is the difference between target and current sensor value 
     * (in sensor units per 100ms). If using motion profiling or Motion Magic, closed loop error is calculated against the current target, 
     * and not the "final" target at the end of the profile/movement. See Phoenix-Documentation information on units.
     * @return Double precision units of error
     */
    public double getTurnError() {
        return turn.getClosedLoopError(0);
    }

    /**
     * Stops both turning and driving by setting their respective motor power to 0.
     */
    public void stopBoth() {
        setDrivePower(0);
        setTurnPower(0);
    }

    /**
     * Sets the brake mode for the motor controller
     * @param device String of either "turn" or "drive" indicating which device to set
     * @param brake Boolean indicating if the brake mode should be set to brake (true) or coast (false)
     */
    public void setBrakeMode(String device, boolean brake) {
        switch (device) {
            case "turn": //turn is a TalonSRX
                if (brake) {
                    turn.setNeutralMode(NeutralMode.Brake);
                } else {
                    turn.setNeutralMode(NeutralMode.Coast);
                }
                break;
            case "drive": //drive is a TalonFX
                if (brake) {
                    drive.setNeutralMode(NeutralMode.Brake);
                } else {
                    drive.setNeutralMode(NeutralMode.Coast);
                }
                break;
        }
    }

    public String getModuleName() {
        return moduleName;
    }

    /**
     * This function is used to output data to the dashboard for debugging the module, typically done in the {@link DriveSubsystem} periodic.
     */
    public void updateDashboard() {
        Dashboard.DriveTrain.setTurnPosition(moduleName, (int) turn.getSelectedSensorPosition(0) & 0x3FF);
        Dashboard.DriveTrain.setTurnSetpoint(moduleName, (int) turn.getClosedLoopTarget(0) & 0x3FF);
        Dashboard.DriveTrain.setTurnPositionError(moduleName, turn.getClosedLoopError(0));
        Dashboard.DriveTrain.setTurnVelocity(moduleName, turn.getSelectedSensorVelocity(0));
        Dashboard.DriveTrain.setTurnZeroPosition(moduleName, getZeroPositionTicks()); 
        // Dashboard.DriveTrain.setTurnPositionErrorChange(moduleName, turn.getErrorDerivative(0));
        Dashboard.DriveTrain.setDriveVelocity(moduleName, drive.getSelectedSensorVelocity(0));
        Dashboard.DriveTrain.setDriveDistance(moduleName, getDriveDistanceMeters());
    }

    /**
     * calculate the turning motor setpoint based on the desired angle and the current angle measurement
     * @param targetAngle desired target in radians
     * @param currentAngle current angle in radians
     * @return Delta angle in radians
     */
    public double deltaAdjustedAngle(double targetAngle, double currentAngle) {
        return ((targetAngle - currentAngle + 180) % 360 + 360) % 360 - 180;
    }

    public double getDriveDistanceMeters() {
        return Helpers.General.encoderToMeters(drive.getSelectedSensorPosition(), this.driveWheelDiam);
    }

    public void resetDistance() {
        drive.setSelectedSensorPosition(0.0);
    }

    public void resetEncoders() {
        // this resets cumulative rotation counts to zero, resets the position of the turn encoders
        // primarily used with an external encoder such as cancoder, does nothing with our lampreys
    }

    public void syncTurningEncoders() {
        // turn.setSelectedSensorPosition(turn.getSelectedSensorPosition(0),1,0);
    }
}