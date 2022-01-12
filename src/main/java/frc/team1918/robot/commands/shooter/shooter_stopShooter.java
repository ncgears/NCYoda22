/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team1918.robot.commands.shooter;

import frc.team1918.robot.Constants;
import frc.team1918.robot.Helpers;
import frc.team1918.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * A command that ...
 */
public class shooter_stopShooter extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterSubsystem m_shooter;

  /**
   * @param subsystem The subsystem used by this command.
   */
  public shooter_stopShooter(ShooterSubsystem subsystem) {
    m_shooter = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //stop the shooter
    Helpers.Debug.debug("Shooter: Stop Shooter");
    m_shooter.runFeeder(false);
    m_shooter.raiseHood(!Constants.Air.AIR_HOOD_UP);
    m_shooter.setShooterSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
