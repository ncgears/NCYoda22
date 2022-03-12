package frc.team1918.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team1918.robot.Helpers;
import frc.team1918.robot.subsystems.VisionSubsystem;

public class vision_setRinglight extends CommandBase {
  // @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"}) //Dont add "unused" under normal operation
  private final VisionSubsystem m_vision;
  private final boolean m_enabled;

  /**
   * @param subsystem The subsystem used by this command.
   */
  public vision_setRinglight(VisionSubsystem subsystem, boolean enabled) {
    m_vision = subsystem;
    m_enabled = enabled;
    // Use addRequirements() here to declare subsystem dependencies.
    //   addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    String status = (m_enabled)?"On":"Off";
    Helpers.Debug.debug("Vision: Ringlight " + status);
    m_vision.setRinglight(m_enabled);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Helpers.Debug.debug("Vision: Ringlight Off");
    m_vision.setRinglight(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}