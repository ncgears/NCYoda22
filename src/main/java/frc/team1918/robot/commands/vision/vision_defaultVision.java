
package frc.team1918.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
//import constants and subsystem
import frc.team1918.robot.Constants;
import frc.team1918.robot.subsystems.VisionSubsystem;

/**
 * A command that runs the drive actions. This passes the OI inputs on to the appropriate drive system (fieldCentricDrive or humanDrive).
 * fieldCentricDrive is simply a call to humanDrive after gyro corrections are made.
 */
public class vision_defaultVision extends CommandBase {
  private final VisionSubsystem m_vision;


  /**
   * Creates a new vision_defaultVision.
   *
   * @param subsystem The vision subsystem this command wil run on.

   */
  public vision_defaultVision(VisionSubsystem subsystem) {
    m_vision = subsystem;
  
    addRequirements(m_vision);
  }

  @Override
  public void execute() {
    m_vision.getVisionInfo();
  }
}