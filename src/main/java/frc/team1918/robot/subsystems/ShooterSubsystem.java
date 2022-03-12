
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
  private double m_shooter_rps = 0.0; // Current shooter speed
  private double m_shooter_oldrps = 0.0; // Previous shooter speed
  private double m_shooter_rpm = 0.0; 
  private Solenoid m_hood;
  public enum namedShots {DEFAULT, LOW, BUMPER, LINE, NONE;}
  public namedShots currentShotName = namedShots.NONE;
  /**
   * Creates a new ExampleSubsystem.
   */
  public ShooterSubsystem() {
    //Setup the SparkMAX controller as desired
    shoot = new WPI_TalonFX(Constants.Shooter.id_Motor1);
    shoot.configFactoryDefault();
    shoot.set(ControlMode.PercentOutput, 0);
    shoot.setNeutralMode(NeutralMode.Coast);
    //shoot.setSensorPhase(Constants.Shooter.isSensorInverted_Motor1);
    shoot.setInverted(Constants.Shooter.isInverted_Motor1);
    shoot.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    shoot.config_kP(0, Constants.Shooter.kP);
    shoot.config_kI(0, Constants.Shooter.kI);
    shoot.config_kD(0, Constants.Shooter.kD);
    shoot.config_IntegralZone(0, Constants.Shooter.kIZone);
    // shoot.configNominalOutputForward(0);
    // shoot.configNominalOutputReverse(0);
    shoot.configPeakOutputForward(1);
    shoot.configPeakOutputReverse(0); //no reverse output
    //Setup the Preshooter
    preShooter = new WPI_TalonSRX(Constants.Shooter.id_Motor2);
    preShooter.configFactoryDefault();
    preShooter.set(ControlMode.PercentOutput, 0);
    preShooter.setNeutralMode(NeutralMode.Coast);
    // preShooter.setInverted(Constants.Shooter.isInverted_Motor2);
    //Setup the solenoid
    m_hood = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Air.id_HoodSolenoid);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run and change the shooter speed if requested
    if (m_shooter_rps != m_shooter_oldrps) {
      shoot.set(ControlMode.Velocity, Helpers.General.rpsToTicksPer100ms(m_shooter_rps, Constants.Shooter.kEncoderFullRotation, Constants.Shooter.kShooterReductionFactor)); //Set the target
      m_shooter_oldrps=m_shooter_rps;
    }
    updateDashboard();
  }

  public void updateDashboard() {
    // Dashboard.Shooter.setCurrentSpeed(getShooterSpeedRPS());
    Dashboard.Shooter.setTargetSpeed(m_shooter_rps);
    Dashboard.Shooter.setHoodPosition(m_hood.get());
    Dashboard.Shooter.setShotName(currentShotName.toString());
  }

  public void setShotName(namedShots shotName) {
    currentShotName = shotName;
  }

  public double getShooterSpeedRPS() {
    // double rawrps = shoot.getSensorCollection().getIntegratedSensorVelocity(); //This works
    double rawrps = shoot.getSelectedSensorVelocity(0);
    double rps = Helpers.General.roundDouble(Helpers.General.ticksPer100msToRPS(rawrps, Constants.Shooter.kEncoderFullRotation, Constants.Shooter.kShooterReductionFactor),0);
    //Helpers.Debug.debug("shooter_rpm="+rpm,1000);
    return rps;
  }

  public void setShooterSpeed(double RPS) {
    RPS = Math.min(RPS, Constants.Shooter.kMaxShooterSpeed);
    RPS = Math.max(RPS, Constants.Shooter.kMinShooterSpeed);
    m_shooter_rps = RPS;
  }

  public void startPreShooter() {
    preShooter.set(ControlMode.PercentOutput,(Constants.Shooter.isInverted_Motor2) ? -Constants.Shooter.kPreShooterSpeed : Constants.Shooter.kPreShooterSpeed);
  }

  public void stopPreShooter() {
    preShooter.set(ControlMode.PercentOutput,0);
  }

  public void stopShooter() {
    shoot.set(ControlMode.PercentOutput,0); //just apply 0 power and let it coast.
    m_shooter_oldrps = 0;
    m_shooter_rps = 0;
    m_shooter_rpm = 0;
  }

  public void increaseShooterSpeed() {
    m_shooter_rps = Math.min(m_shooter_rps + Constants.Shooter.kSpeedIncrementSize, Constants.Shooter.kMaxShooterSpeed);
    m_shooter_rpm = m_shooter_rps * 60;
    Helpers.Debug.debug("New Shooter Speed:"+m_shooter_rps+"/"+m_shooter_rpm);
  }

  public void decreaseShooterSpeed() {
    m_shooter_rps = Math.max(m_shooter_rps - Constants.Shooter.kSpeedIncrementSize, Constants.Shooter.kMinShooterSpeed);
    m_shooter_rpm = m_shooter_rps * 60;
    Helpers.Debug.debug("New Shooter Speed:"+m_shooter_rps+"/"+m_shooter_rpm);
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
