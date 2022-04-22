
package frc.team1918.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.team1918.robot.Constants;
import frc.team1918.robot.Dashboard;
import frc.team1918.robot.Helpers;
import frc.team1918.robot.RobotContainer;
import frc.team1918.robot.commandgroups.cg_djRumble;
import frc.team1918.robot.subsystems.ShooterSubsystem.namedShots;

import com.kauailabs.navx.frc.AHRS;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
// import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class VisionSubsystem extends SubsystemBase {
  NetworkTable table;
  double t1x, t1y, t1s; //target 1
  double t2x, t2y, t2s; //target 2
  double t3x, t3y, t3s; //target 3
  String t1color, t2color, t3color;
  double defaultValue = 0.0;
  String defaultColor = "none";
  double desiredAngle;
  boolean angleLocked = false;
  double totalPixel =500;
  double FOV = 60;
  Relay m_ringlight = new Relay(Constants.Vision.id_RingLight);
  PhotonCamera m_camera = new PhotonCamera("gloworm");
  double photonLatency = 0.0;
  double m_pitch = -100.0;
  // Command m_rumbleCommand = new cg_djRumble(this);

  private static AHRS m_gyro = new AHRS(SPI.Port.kMXP);
  
  //horizontal field of view/ diagonal field of view 68.5 degrees

  public VisionSubsystem() {
    table = NetworkTableInstance.getDefault().getTable("VisionInfo");
  }
  
  @Override
  public void periodic() {

    updateDashboard();

    /* //Disabled all telemetry for ball detection/intake vision
    // This method will be called once per scheduler run and change the shooter speed if requested
    double t1x = table.getEntry("target1x").getDouble(defaultValue);
    double t1y = table.getEntry("target1y").getDouble(defaultValue);
    String t1color = table.getEntry("target1color").getString(defaultColor);
    double t1s = table.getEntry("target1size").getDouble(defaultValue);
    double t2x = table.getEntry("target2x").getDouble(defaultValue);
    double t2y = table.getEntry("target2y").getDouble(defaultValue);
    String t2color = table.getEntry("target1color").getString(defaultColor);
    double t2s = table.getEntry("target2size").getDouble(defaultValue);
    double t3x = table.getEntry("target3x").getDouble(defaultValue);
    double t3y = table.getEntry("target3y").getDouble(defaultValue);
    String t3color = table.getEntry("target1color").getString(defaultColor);
    double t3s = table.getEntry("target3size").getDouble(defaultValue);

    Object[][] targets = { //create a multi-dimensional array
      {t1x, t1y, t1color, t1s}, 
      {t2x, t2y, t2color, t2s}, 
      {t3x, t3y, t3color, t3s}
    };
    for (int i=0; i<1; i++) {
      Object[] target = targets[i];
      // Helpers.Debug.debug("Vision: T"+(i+1)+"(x:"+target[0]+" y:"+target[1]+" color: "+target[2]+" size: "+target[3]+")");
    }
    // Helpers.Debug.debug(Double.toString(((t1x-250)/250)*FOV));
    */
  }

  public void updateDashboard() {
    Dashboard.Vision.setVisionRinglight(m_ringlight.get()==Value.kReverse);
  }

  public void lockAngle() {
		desiredAngle = Helpers.General.roundDouble(m_gyro.getAngle(), 3);
		angleLocked = true;
		Helpers.Debug.debug("Angle Locked to "+desiredAngle);
	}

	public void unlockAngle() {
		if(angleLocked) Helpers.Debug.debug("Angle Unlocked");
		angleLocked = false;
	}
  
  public double calcAngleStraight() {
    double kP = Constants.Vision.kErrorCorrection_P;
		double errorAngle = ((Math.abs(t1x-250)/250)*30);
    // double errorAngle = Math.toRadians((Math.abs(t1x-250)/250)*FOV);
// 
		double correction = errorAngle * kP;
		return correction;
  }
  // public void drive(){
    
  // if(rot == 0) { //We are not applying rotation
  //   if(!angleLocked) { //We havent locked an angle, so lets save the desiredAngle and lock it
  //     lockAngle();
  //   } else {
  //     if (Math.abs(fwd) > 0 || Math.abs(str) > 0) { //Only do angle correction while moving, for safety reasons
  //       rot += calcAngleStraight(); //Add some correction to the rotation to account for angle drive
  //     }
  //   }
  // }
  
  /**
   * This enables or disables the ring light
   * @param enabled - true to turn on light, false to turn it off
   */
  public void setRinglight(boolean enabled) {
    // m_ringlight.set((enabled) ? Value.kReverse : Value.kOff);
    m_camera.setLED((enabled) ? VisionLEDMode.kOn : VisionLEDMode.kOff);
  }

  /**
   * This sets the desired color for vision tracking
   * @param color - color of balls to track: "blue", "red", "both", or "none"
   */
  public void setDesiredColor(String color) {
    String desired ;

    switch (color.toLowerCase()) {
      case "blue":
      case "red":
      case "both":
        desired = color.toLowerCase();
        break;
      default:
        desired = "none";
    }
    // desired = "blue";
    table.getEntry("desiredColor").setString(desired);
  }

  public double getVisionTurn() {
    //-1.0 .. 0.0 .. 1.0 == ccw .. neutral .. cw
    double fovLimit = 29.0; //+ or - yaw from center
    double turn = 0.0;
    var result = m_camera.getLatestResult();
    if(result.getLatencyMillis() == photonLatency) { //same as last loop, assume we lost photon
      return 0.0;
    }
    photonLatency = result.getLatencyMillis();
    SmartDashboard.putBoolean("Vision/HasTargets", result.hasTargets());
    if (result.hasTargets()) {
      var target = result.getBestTarget().getYaw() + Constants.Vision.kOffsetDegrees;
      if(Math.abs(target) >= fovLimit) {
        Helpers.Debug.debug("Vision: target outside fov limit");
        turn = 0.0; //over 15deg then skip aiming
        m_pitch = -100.0;
      } else {
        turn = target/fovLimit;
        turn = (Math.abs(turn) >= 0.05) ? Math.max(Constants.Vision.kMinTurnPower,Math.abs(turn)) * Math.signum(turn) : 0.0; //minimum turn speed
        m_pitch = result.getBestTarget().getPitch();
      }
      // SmartDashboard.putNumber("Vision/turnControl",turn);
      // SmartDashboard.putNumber("Vision/targetPitch",m_pitch);
    } else {
      turn = 0.0;
      m_pitch = -100.0;
    }
    return turn;
  }

  // public void updateVisionPitch() {
  //   double pitch = -100.0;
  //   var result = m_camera.getLatestResult();
  //   if(result.getLatencyMillis() == photonLatency) { //same as last loop, assume we lost photon
  //     m_pitch = -100.0;
  //   }
  //   photonLatency = result.getLatencyMillis();
  //   SmartDashboard.putBoolean("Vision/HasTargets", result.hasTargets());
  //   if (result.hasTargets()) {
  //     pitch = result.getBestTarget().getPitch();
  //   } else {
  //     pitch = -100.0;
  //   }
  //   SmartDashboard.putNumber("Vision/Pitch",pitch);
  //   if (pitch != 0.0) m_pitch = pitch;
  // }

  public double getVisionPitch() {
    return m_pitch;
  }

  public namedShots selectShot(double pitch) {
    Helpers.Debug.debug("Vision: Selecting shot for pitch "+pitch);
    final double pitchMin = -2.0;
    final double pitchMaxProtected = 4.2;
    final double pitchMaxWall = 11.8;
    final double pitchMaxLine = 20; //TBD
    final double pitchMaxTarmac = 21.5; //TBD
    final double pitchMax = 21.8; //TBD
    if(pitch < pitchMin || pitch > pitchMax) {
      Helpers.Debug.debug("Vision: too close/far for auto shot selection");
      // Helpers.OI.rumble(true);
      // m_rumbleCommand.schedule();
      return namedShots.NONE;
    }

    // else if (pitch < pitchMaxTarmac) { //Tarmac shot
    //   Helpers.Debug.debug("Vision: Auto selecting TARMAC shot");
    //   return namedShots.TARMAC;
    // } 
    else if (pitch < pitchMaxProtected) { //Protected shot
      Helpers.Debug.debug("Vision: Auto selecting PROTECTED shot");
      return namedShots.PROTECTED;
    } else if (pitch < pitchMaxWall){ //Wall shot
      Helpers.Debug.debug("Vision: Auto selecting WALL shot");
      return namedShots.WALL;
    } else if (pitch < pitchMaxLine) { //Line shot
      Helpers.Debug.debug("Vision: Auto selecting LINE shot");
      // m_rumbleCommand.schedule();
      return namedShots.LINE;
    } else {
      return namedShots.NONE;
    }
  }

}
