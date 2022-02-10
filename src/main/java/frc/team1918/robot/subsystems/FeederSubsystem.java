
package frc.team1918.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1918.robot.Constants;

public class FeederSubsystem extends SubsystemBase {
  private WPI_TalonFX m_feeder; // feeder controller

//TODO: See https://www.chiefdelphi.com/t/code-for-ir-break-beam/396373/4 for beam break examples and triggers

  /**
   * Creates a new ExampleSubsystem.
   */
  public FeederSubsystem() {
 //Setup the SparkMAX controller as desired
    m_feeder = new WPI_TalonFX(Constants.Feeder.id_Motor1);
    m_feeder.configFactoryDefault();
    m_feeder.set(ControlMode.PercentOutput, 0);
    m_feeder.setNeutralMode(NeutralMode.Brake);
    m_feeder.setInverted(Constants.Feeder.isInverted_Motor1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * This function runs the feeder
   * @param fwd - true to run forward, false to run reverse
   */
  public void runFeeder(boolean fwd) {
    //send command to run Feeder
    m_feeder.set(ControlMode.PercentOutput, (fwd) ? Constants.Feeder.speed_Motor1 : -Constants.Feeder.speed_Motor1);
    //Dashboard.Feeder.setHoodPosition(up);
  }

  public void stopFeeder() {
    m_feeder.set(ControlMode.PercentOutput,0);
  }

  public static boolean hasFirstBall() {
    //This would use the beam break logic to figure these out
    return false;
  }

  public static boolean hasSecondBall() {
    //This would use the beam break logic to figure these out
    return false;
  }
}
