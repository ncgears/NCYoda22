
package frc.team1918.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
//import subsystem
import frc.team1918.robot.subsystems.DriveSubsystem;

/**
 * A command to reset the gyro to 0.
 * This happens when we have the robot in a known orientation to allow us to track the orientation of the robot.
 * @implNote The reset happens during the end method of the command to ensure that it always executes even if the command completes before the gyro finishes executing the command.
 */
public class drive_resetGyro extends CommandBase {
  private final DriveSubsystem m_drive;

  /**
   * @param subsystem The drive subsystem this command will run on.
   */
  public drive_resetGyro(DriveSubsystem subsystem) {
    m_drive = subsystem;
    addRequirements(m_drive);
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.zeroHeading();
    // resetGyro();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}