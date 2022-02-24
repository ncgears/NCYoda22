
package frc.team1918.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1918.robot.Constants;
import frc.team1918.robot.Dashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


public class ClimberSubsystem extends SubsystemBase {
  private WPI_TalonSRX climber_1;
  private WPI_TalonSRX climber_2;
  private Solenoid hook_release_1;
  private Solenoid hook_release_2;
  private Solenoid whirlySolenoid; 
  private DigitalInput m_limitSwitchLeft; //First Beam Break (at intake)
  private DigitalInput m_limitSwitchRight;

  public ClimberSubsystem() {
    m_limitSwitchLeft = new DigitalInput(Constants.Feeder.id_BeamBreak1);
    m_limitSwitchRight = new DigitalInput(Constants.Feeder.id_BeamBreak2);
    climber_1 = new WPI_TalonSRX(Constants.Climber.id_Motor1);
    climber_2 = new WPI_TalonSRX(Constants.Climber.id_Motor2);
    hook_release_1 = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Air.id_ClimbHook1Solonoid);
    hook_release_2 = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Air.id_ClimbHook2Solonoid);
    whirlySolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Air.id_WhirlyGigSolonoid);

    climber_1.configFactoryDefault(); 
    climber_1.set(ControlMode.PercentOutput, 0);
    climber_1.setNeutralMode(NeutralMode.Brake); 
    climber_1.config_kP(0,Constants.Climber.kP); //P value for PID_PRIMARY
    climber_1.config_kI(0,Constants.Climber.kI); //I value for PID_PRIMARY
    climber_1.config_kD(0,Constants.Climber.kD); //D value for PID_PRIMARY
    climber_1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,30);
    climber_1.configFeedbackNotContinuous(Constants.Climber.isSensorNotContinuous, 0);
    climber_1.setSensorPhase(Constants.Climber.isSensorInverted_Motor1); 
    climber_1.setInverted(Constants.Climber.isInverted_Motor1);
    climber_1.setSelectedSensorPosition(0);

    climber_2.configFactoryDefault(); 
    climber_2.setNeutralMode(NeutralMode.Brake); 
    // climber_2.follow(climber_1); //Climber 2 Was Lagging Behind, as well as going the wrong direction
    climber_2.setInverted((Constants.Climber.isInvertedFromMaster_Motor2) ? InvertType.OpposeMaster : InvertType.FollowMaster);
  }

  /**
   * This function raises the Whirlygig. This cannot be undone except for releasing air pressure to reset.
   */
  public void raiseWhirlygig() {
    whirlySolenoid.set(Constants.Air.stateWhirlygigUp);
  }

  /**
   * This sets the hook mode to locked (false) or unlocked (true)
   * @param hook_id - (int) This is the hook id to lock/unlock (1 or 2)
   * @param unlock - (boolean) This indicates if the hook should be unlocked (true)
   */
  public void setHookMode(int hook_id, boolean unlock) {
    switch(hook_id) {
      case 1:
        hook_release_1.set(unlock);
        break;
      case 2:
        hook_release_2.set(unlock);
        break;
    }
  }

  /**
   * This moves the climber to a specific position using the encoder
   * @param target - Encoder position to move to
   */
  public boolean leftLimitSwitchTouch(){
    return m_limitSwitchLeft.get();
  }
  
  public boolean rightLimitSwitchTouch(){
    return m_limitSwitchRight.get();
  }
  public void moveClimberToTarget(int target) {
    climber_1.set(ControlMode.Position,target);
  }

  /**
   * This runs the climber in the forward direction
   */
  public void climberForward() {
    climber_1.set(ControlMode.PercentOutput, Constants.Climber.kClimberSpeed);
    climber_2.set(ControlMode.PercentOutput, Constants.Climber.kClimberSpeed * -1);
    Dashboard.Climber.setClimberDirection("Forward");
  }

  /**
   * This runs the climber in the reverse direction
   */
  public void climberReverse() {
    climber_1.set(ControlMode.PercentOutput, Constants.Climber.kClimberSpeed * -1);
    climber_2.set(ControlMode.PercentOutput, Constants.Climber.kClimberSpeed);
    Dashboard.Climber.setClimberDirection("Reverse");
  }

  /**
   * This stops the climber
   */
  public void climberStop() {
    climber_1.set(ControlMode.PercentOutput, 0);
    Dashboard.Climber.setClimberDirection("Stopped");
  }

  /**
   * This gets the climbers current encoder position
   */
  public double getClimberPosition() {
    return climber_1.getSelectedSensorPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run, usually used for updating dashboard data
    climber_2.follow(climber_1); //Make sure climber_2 is always following climber_1
    Dashboard.Climber.setClimberPosition(getClimberPosition());
    
    if (leftLimitSwitchTouch()){
      // reverse left arm
    }
    if (rightLimitSwitchTouch()){
      // reverse right arm
    }


  }
}
