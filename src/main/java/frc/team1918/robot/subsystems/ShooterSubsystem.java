
package frc.team1918.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1918.robot.Constants;
import frc.team1918.robot.Dashboard;
import frc.team1918.robot.Helpers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class ShooterSubsystem extends SubsystemBase {
  private WPI_TalonFX shoot; // shooter controller
  private WPI_TalonSRX preShooter;
  private double m_shooter_rpm = 0.0; // Current shooter speed
  private double m_shooter_oldrpm = 0.0; // Previous shooter speed
  private Solenoid m_hood;
  /**
   * Creates a new ExampleSubsystem.
   */
  public ShooterSubsystem() {
    //Setup the SparkMAX controller as desired
    shoot = new WPI_TalonFX(Constants.Shooter.id_Motor1);
    shoot.configFactoryDefault();
    shoot.set(ControlMode.PercentOutput, 0);
    shoot.setNeutralMode(NeutralMode.Coast);
    shoot.setInverted(Constants.Shooter.isInverted_Motor1);
    shoot.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    shoot.config_kP(0, Constants.Shooter.kP);
    shoot.config_kI(0, Constants.Shooter.kI);
    shoot.config_kD(0, Constants.Shooter.kD);
    shoot.config_IntegralZone(0, Constants.Shooter.kIZone);
    //Setup the Preshooter
    preShooter = new WPI_TalonSRX(Constants.Shooter.id_Motor2);
    preShooter.configFactoryDefault();
    preShooter.set(ControlMode.PercentOutput, 0);
    preShooter.setNeutralMode(NeutralMode.Coast);
    preShooter.setInverted(Constants.Shooter.isInverted_Motor1);
    //Setup the solenoid
    m_hood = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Air.id_HoodSolonoid);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run and change the shooter speed if requested
    if (m_shooter_rpm != m_shooter_oldrpm) {
      shoot.set(ControlMode.Velocity, Helpers.General.rpmToTicksPer100ms(m_shooter_rpm, Constants.Shooter.kEncoderFullRotation)); //Set the target
      m_shooter_oldrpm=m_shooter_rpm;
    }
    Dashboard.Shooter.setCurrentSpeed(getShooterSpeedRPM());
  }

  public double getShooterSpeedRPM() {
    return Helpers.General.roundDouble(Helpers.General.ticksPer100msToRPM(shoot.getSelectedSensorVelocity(0), Constants.Shooter.kEncoderFullRotation),0);
  }

  public void setShooterSpeed(double RPM) {
    RPM = Math.min(RPM, Constants.Shooter.kMaxShooterSpeed);
    RPM = Math.max(RPM, Constants.Shooter.kMinShooterSpeed);
    m_shooter_rpm = RPM;
  }

  public void startPreShooter() {
    preShooter.set(ControlMode.PercentOutput,Constants.Shooter.kPreShooterSpeed);
  }

  public void stopPreShooter() {
    preShooter.set(ControlMode.PercentOutput,0);
  }

  public void increaseShooterSpeed() {
    m_shooter_rpm = Math.min(m_shooter_rpm + Constants.Shooter.kSpeedIncrementSize, Constants.Shooter.kMaxShooterSpeed);
    Helpers.Debug.debug("New Shooter Speed:"+m_shooter_rpm);
  }

  public void decreaseShooterSpeed() {
    m_shooter_rpm = Math.max(m_shooter_rpm - Constants.Shooter.kSpeedIncrementSize, Constants.Shooter.kMinShooterSpeed);
    Helpers.Debug.debug("New Shooter Speed:"+m_shooter_rpm);
  }

  public void setShooterSpeedFromDashboard() {
    setShooterSpeed(Dashboard.Shooter.getTargetSpeed(0));
  }

  public void raiseHood(boolean up) {
    //send command to air to put hood up or down based on boolean
    m_hood.set(up);
    Dashboard.Shooter.setHoodPosition(up);
  }
}
