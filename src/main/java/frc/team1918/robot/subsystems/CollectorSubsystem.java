
package frc.team1918.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1918.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class CollectorSubsystem extends SubsystemBase {
  private WPI_TalonSRX coll; //collector controller
  private Solenoid m_coll1; //collector solenoid 1
  private boolean m_collector_deployed = false;

  public CollectorSubsystem() {
    coll = new WPI_TalonSRX(Constants.Collector.id_Motor1);
    m_coll1 = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Air.id_CollectorSolenoid);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setIntakeSpeed(double speed) {
    coll.set(ControlMode.PercentOutput, speed);
  }

  public void startIntake(boolean reverse) {
    coll.set(ControlMode.PercentOutput, (reverse) ? -Constants.Collector.kDefaultCollectorSpeed : Constants.Collector.kDefaultCollectorSpeed);
  }

  public void stopIntake() {
    coll.set(ControlMode.PercentOutput, 0);
  }

  public boolean isCollectorDeployed() {
    return m_collector_deployed;
  }

  public void setCollectorPosition(String position) {
    switch(position) {
      case "deploy":
      case "down":
      case "out":
        //deploy
        m_coll1.set(!Constants.Air.stateCollectorDeployed);
        m_collector_deployed = true;
        break;
      case "retract":
      case "up":
      case "in":
      case "stow":
      default:
        //retract/stow
        m_coll1.set(Constants.Air.stateCollectorDeployed);
        m_collector_deployed = false;
        break;
    }
  }
}
