
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
  private boolean m_collector_down = false;

  public CollectorSubsystem() {
    coll = new WPI_TalonSRX(Constants.Collector.id_Motor1);
    m_coll1 = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Air.id_CollectorSolonoid);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setIntakeSpeed(double speed) {
    coll.set(ControlMode.PercentOutput, speed);
  }

  public boolean isCollectorDown() {
    return m_collector_down;
  }

  public void setCollectorPosition(String position) {
    switch(position) {
      case "down":
        //put collector down
        m_coll1.set(!Constants.Air.stateCollectorUp);
        m_collector_down = true;
        break;
      case "up":
      case "stow":
      default:
        //put collector up
        m_coll1.set(Constants.Air.stateCollectorUp);
        m_collector_down = false;
        break;
    }
  }
}
