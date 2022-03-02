
package frc.team1918.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1918.robot.Constants;
import frc.team1918.robot.Helpers;

public class FeederSubsystem extends SubsystemBase {
  private WPI_TalonFX m_feeder; // feeder controller
  private DigitalInput m_beam_intake; //First Beam Break (at intake)
  private DigitalInput m_beam_shooter; //Second Beam Break (before shooter)
  private DigitalInput m_feeder_switch; //Shoe switch

//TODO: See https://www.chiefdelphi.com/t/code-for-ir-break-beam/396373/4 for beam break examples and triggers

  /**
   * Creates a new ExampleSubsystem.
   */
  public FeederSubsystem() {
    // Setup the beam breaks
    m_beam_intake = new DigitalInput(Constants.Feeder.id_BeamBreak1);
    m_beam_shooter = new DigitalInput(Constants.Feeder.id_BeamBreak2);
    m_feeder_switch = new DigitalInput(Constants.Feeder.id_FeederSwitch);
    // Setup the feeder
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

  public boolean hasFirstBall() {
    //We have a ball by the intake
    return !m_beam_intake.get();
    // return false;
  }

  public boolean hasSecondBall() {
    //We have a ball up by the shooter
    // Helpers.Debug.debug("Sensor="+m_beam_shooter.get(),1000);
    return !m_beam_shooter.get();
    // return false;
  }

  public boolean hasBall() {
    // if(!m_feeder_switch.get()) Helpers.Debug.debug("Feeder: has a ball");
    return !m_feeder_switch.get();
  }
}
