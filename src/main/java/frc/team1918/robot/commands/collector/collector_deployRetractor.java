/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team1918.robot.commands.collector;

import frc.team1918.robot.Helpers;
import frc.team1918.robot.subsystems.CollectorSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * A command that ...
 */
public class collector_deployRetractor extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"}) //Dont add "unused" under normal operation
  private final CollectorSubsystem m_collector;
  private final boolean m_retracted;

  /**
   * @param subsystem The subsystem used by this command.
   */
  public collector_deployRetractor(CollectorSubsystem subsystem, boolean retracted) {
    m_collector = subsystem;
    m_retracted = retracted;
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Helpers.Debug.debug("Collector: Deploy Retractor");
    m_collector.setRetractorPosition(m_retracted);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
