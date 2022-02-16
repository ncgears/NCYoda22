
package frc.team1918.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.team1918.robot.Constants;
import frc.team1918.robot.Helpers;


public class VisionSubsystem extends SubsystemBase {
  NetworkTable table;
  double t1x, t1y, t1s; //target 1
  double t2x, t2y, t2s; //target 2
  double t3x, t3y, t3s; //target 3
  String t1color, t2color, t3color;
  double defaultValue = 0.0;
  String defaultColor = "none";

  public VisionSubsystem() {
    table = NetworkTableInstance.getDefault().getTable("VisionInfo");
  }
  
  @Override
  public void periodic() {
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
    double t2s = table.getEntry("target3size").getDouble(defaultValue);

    Object[][] targets = { //create a multi-dimensional array
      {t1x, t1y, t1color, t1size}, 
      {t2x, t2y, t2color, t2size}, 
      {t3x, t3y, t3color, t3size}
    };

    for (int i=0; i<targets.length; i++) {
      Object[] target = targets[i];
      Helpers.Debug.debug("Vision: T"+(i+1)+"(x:"+target[0]+" y:"+target[1]+" color: "+target[2]+" size: "+target[3]+")");
    }
  }

  public void setDesiredColor(String color) {
    String desired;
    switch (color.toLowerCase()) {
      case "blue":
      case "red":
      case "both":
        desired = color.toLowerCase();
        break;
      default:
        desired = "none";
    }
    table.getEntry("desiredColor").setString(desired);
  }

}
