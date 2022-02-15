
package frc.team1918.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.team1918.robot.Constants;
import frc.team1918.robot.Helpers;
import frc.team1918.robot.commands.drive.drive_defaultDrive;
import frc.team1918.robot.commands.helpers.helpers_debugMessage;


public class VisionSubsystem extends SubsystemBase {
  public VisionSubsystem() {
    
    
  }
  
NetworkTable table;
double[] areas;
double[] defaultValue = new double[0];


public void getVisionInfo() {
 
  // NetworkTable table = NetworkTableInstance.getDefault().getTable("VisionInfo");
  // NetworkTableEntry ballCoordinate = table.getEntry("ballCoordinates1");
  // System.out.println(ballCoordinate);
  // Helpers.Debug.debug("Vision output: "+ballCoordinate.toString());
  Helpers.Debug.debug("Vision output: ");

    // double[] areas = table.getEntry("area").getDoubleArray(defaultValue);

    // System.out.print("areas: " );

    // for (double area : areas) {
    //   System.out.print(area + " ");
    // }

    // System.out.println();
}

 
}
