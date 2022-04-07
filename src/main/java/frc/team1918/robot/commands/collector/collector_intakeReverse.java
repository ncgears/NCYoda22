/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team1918.robot.commands.collector;

import frc.team1918.robot.Helpers;
import frc.team1918.robot.subsystems.CollectorSubsystem;
import frc.team1918.robot.subsystems.CollectorSubsystem.intakeDirection;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * A command that ...
 */
public class collector_intakeReverse extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"}) //Dont add "unused" under normal operation
  private final CollectorSubsystem m_collector;

  /**
   * @param subsystem The subsystem used by this command.
   */
  public collector_intakeReverse(CollectorSubsystem subsystem) {
    m_collector = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Helpers.Debug.debug("Collector: Intake Reverse");
    m_collector.startIntake(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(m_collector.currentIntakeDirection == intakeDirection.FORWARD) {
      Helpers.Debug.debug("Collector: Intake Forward");
      m_collector.startIntake(false);
    } else {
      Helpers.Debug.debug("Collector: Intake Stop");
      m_collector.stopIntake();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
