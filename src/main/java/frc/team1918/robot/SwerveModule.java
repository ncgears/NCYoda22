
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
    private double wheelOffsetMM = 0;
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
        drive = new WPI_TalonFX(moduleConstants.idDriveMotor);
        turn = new WPI_TalonSRX(moduleConstants.idTurnMotor);
        isDrivePowerInverted = false;
        TURN_P = moduleConstants.turnP;
        TURN_I = moduleConstants.turnI;
        TURN_D = moduleConstants.turnD;
        TURN_IZONE = moduleConstants.turnIZone;
        TURN_ALLOWED_ERROR = moduleConstants.turnMaxAllowedError;

        turn.configFactoryDefault(); //Reset controller to factory defaults to avoid wierd stuff from carrying over
        turn.set(ControlMode.PercentOutput, 0); //Set controller to disabled
        turn.setNeutralMode(NeutralMode.Brake); //Set controller to brake mode
        turn.configSelectedFeedbackSensor(  FeedbackDevice.Analog, //  FeedbackDevice.CTRE_MagEncoder_Absolute, // Local Feedback Source
                                            Constants.Global.PID_PRIMARY,				// PID Slot for Source [0, 1]
                                            Constants.Global.kTimeoutMs);				// Configuration Timeout
        turn.configFeedbackNotContinuous(Constants.Global.SWERVE_SENSOR_NONCONTINUOUS, 0); //Disable continuous feedback tracking (so 0 and 1024 are effectively one and the same)
       
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
        turn.set(ControlMode.Position, 0); //TODO: Add constant for home_on_init

        drive.configFactoryDefault();
        drive.set(ControlMode.PercentOutput, 0);
        drive.setNeutralMode(NeutralMode.Brake);
        drive.setInverted(moduleConstants.driveIsInverted);
        drive.config_kP(0, 0.0005);
        drive.config_kI(0, 0.0);
        drive.config_kD(0, 0.00005);
        drive.config_IntegralZone(0, 4740);

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
        double rawRpm = drive.getSelectedSensorVelocity();  //getEncoder().getVelocity();
        double wheelRpm = Helpers.General.gearCalcDouble(rawRpm,Constants.DriveTrain.DT_DRIVE_FIRST_GEARONE,
            Constants.DriveTrain.DT_DRIVE_FIRST_GEARTWO,
            Constants.DriveTrain.DT_DRIVE_SECOND_GEARONE,
            Constants.DriveTrain.DT_DRIVE_SECOND_GEARTWO);
        double wheelDiam = Constants.DriveTrain.DT_WHEEL_DIAM_MM - this.wheelOffsetMM;
        return new SwerveModuleState(Helpers.General.rpmToMetersPerSecond(wheelRpm, wheelDiam), getTurnPositionAsRotation2d());
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
        Rotation2d delta = desiredState.angle.minus(currentAngle);
        if (Math.abs(delta.getDegrees()) > 90.0 && Math.abs(delta.getDegrees()) < 270) { //new requested delta is between 90 and -90 (270) degrees, invert drive speed and rotate 180 degrees from desired
            return new SwerveModuleState(-desiredState.speedMetersPerSecond, desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0)));
        } else { //no optimization necessary
            return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
        }
    }

    /**
     * This function takes a desiredState and instructs the motor controllers to move based on the desired state
     * @param desiredState Desired state with speed and angle.
     * FL = 5570, FR = 5200, RL = 5200, RR = 4740
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        double wheelDiam = Constants.DriveTrain.DT_WHEEL_DIAM_MM - this.wheelOffsetMM;
        SwerveModuleState state = (Constants.Swerve.USE_OPTIMIZATION) ? optimize(desiredState) : desiredState;
        if (Constants.Swerve.USE_DRIVE_PID) {
            // TODO: Send speed to closed loop control rather than percent output. Enable USE_DRIVE_PID and test.
            double motorRpm = (Helpers.General.metersPerSecondToRPM(state.speedMetersPerSecond, wheelDiam) / Constants.DriveTrain.DT_DRIVE_CONVERSION_FACTOR);
            // Helpers.Debug.debug(moduleName+" desired mps: "+state.speedMetersPerSecond+" motorRpm: "+motorRpm);
            drive.set(ControlMode.Velocity, motorRpm, DemandType.ArbitraryFeedForward, feedforward.calculate(state.speedMetersPerSecond));
        } else {
            // old way, which was not correct
            // drive.set(state.speedMetersPerSecond);
            double percentOutput = state.speedMetersPerSecond / Constants.Swerve.kMaxSpeedMetersPerSecond;
            drive.set(ControlMode.PercentOutput, percentOutput);
        }
        
        //Determine which direction we should turn to get to the desired setpoint
        int cur_ticks = getTurnPosition();
        int min_ticks = minTurnTicks(Helpers.General.radiansToTicks(state.angle.getRadians()), cur_ticks);
        int turn_ticks = min_ticks + cur_ticks;
        // Helpers.Debug.debug("Encoder Ticks: " +cur_ticks);
        if(Helpers.Debug.debugThrottleMet(debug_ticks1)) {
            Helpers.Debug.debug(moduleName+" Speed (metersPerSecond)="+Helpers.General.roundDouble(state.speedMetersPerSecond,3)+" Turn Setpoint="+turn_ticks);
        }
        debug_ticks1++;

        turn.set(ControlMode.Position, turn_ticks); //Set the turn setpoint
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

    /**
     * Returns the raw units of the current location of the turn sensor
     * @return (integer) Raw units (typically encoder ticks) of turn location
     */
    public int getTurnPosition(){
        return ((int) turn.getSelectedSensorPosition(0) & 0x3FF);
        //return (turn.getSensorCollection().getAnalogIn()); //This gets an ADC value for the analog sensor
        //We may want to use a bitwise "AND" with 0x3FF (1023), 0x7FF (2047), 0xFFF (4095) to get just the most significant bits that we are interested in.
        //Explanation: & is a bitwise "AND" operator, and 0xFFF is 4095 in Hex, consider "0101010101 AND 1111 = 0101"
    }

    /**
     * Returns a rotation2d object representing the current location of the turn sensor
     * @return Rotation2d object of the current position
     */
    public Rotation2d getTurnPositionAsRotation2d(){
        return new Rotation2d(Helpers.General.ticksToRadians(getTurnPosition()));
    }

    /**
	 * Set turn to pos from 0 to 1 using PID using shortest turn to get the wheels aimed the right way
	 * @param wa wheel angle location to set to in radians
	 */
	public void setTurnLocation(double waRads) {
        double currentAngleRads = Helpers.General.ticksToRadians(getTurnPosition());
        double targetAngleRads = waRads;
        int currentNumRotations = (int) (currentAngleRads / FULL_ROTATION);
        targetAngleRads += (currentNumRotations >= 0) ? currentNumRotations * FULL_ROTATION : (currentNumRotations + 1) * FULL_ROTATION;

        if ((targetAngleRads > currentAngleRads + FULL_ROTATION * 0.25) || (targetAngleRads < currentAngleRads - FULL_ROTATION * 0.25)) { //if target is more than 25% of a rotation either way
            if (currentAngleRads < targetAngleRads) { //left strafe
                if (targetAngleRads - currentAngleRads > FULL_ROTATION * 0.75) { //if target would require moving less than 75% of a rotation, just go there
                    targetAngleRads -= FULL_ROTATION;
                } else { //otherwise, turn half a rotation from the target and reverse the drive power
                    targetAngleRads -= FULL_ROTATION * 0.5;
                    this.isDrivePowerInverted = true;
                }
            } else { //right strafe
                if ( currentAngleRads - targetAngleRads > FULL_ROTATION * 0.75) { //if target would require moving less than 75% of a rotation, just go there
                    targetAngleRads += FULL_ROTATION;
                } else { //otherwise, turn half a rotation from the target and reverse the drive power
                    targetAngleRads += FULL_ROTATION * 0.5;
                    this.isDrivePowerInverted = true;
                }
            }
        }
        turn.set(ControlMode.Position,targetAngleRads);
        // System.out.println(moduleName + " setTurnLocation="+targetAngle+"; isDrivePowerInverted="+this.isDrivePowerInverted);
    }

    /**
     * Gets the closed-loop error. The units depend on which control mode is in use. If closed-loop is seeking a target sensor position, closed-loop error is the difference between target and current sensor value (in sensor units. Example 4096 units per rotation for CTRE Mag Encoder). If closed-loop is seeking a target sensor velocity, closed-loop error is the difference between target and current sensor value (in sensor units per 100ms). If using motion profiling or Motion Magic, closed loop error is calculated against the current target, and not the "final" target at the end of the profile/movement. See Phoenix-Documentation information on units.
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
     * Stops the drive by setting the motor power to 0.
     */
    public void stopDrive() {
        setDrivePower(0);
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

    /**
     * Sets the turn power to a specific PercentOutput
     * @param p Double from -1 to 1 indicating the turn power, where 0.0 is stopped
     */
    public void setTurnPowerPercent(double p) {
           turn.set(ControlMode.PercentOutput, p);
    }

    /**
     * Sets the turn position to a specific setpoint using the current encoder (absolute or relative)
     * @param et Encoder Ticks to turn module to.  This depends on which encoder is active.
     */
    public void setTurnLocationInEncoderTicks(double et) {
        // System.out.print(moduleName + " setTurnLocationInEncoderTicks = "+et+"\n");
        turn.set(ControlMode.Position, et);
    }

    /**
     * This function is used to output data to the dashboard for debugging the module, typically done in the {@link DriveSubsystem} periodic.
     */
    public void updateDashboard() {
        Dashboard.DriveTrain.setTurnPosition(moduleName, turn.getSelectedSensorPosition(0));
        Dashboard.DriveTrain.setTurnSetpoint(moduleName, turn.getClosedLoopTarget(0));
        Dashboard.DriveTrain.setTurnPositionError(moduleName, turn.getClosedLoopError(0));
        Dashboard.DriveTrain.setTurnVelocity(moduleName, turn.getSelectedSensorVelocity(0));
        Dashboard.DriveTrain.setTurnPositionErrorChange(moduleName, turn.getErrorDerivative(0));
        Dashboard.DriveTrain.setDriveVelocity(moduleName, drive.getSelectedSensorVelocity(0));
    }
}