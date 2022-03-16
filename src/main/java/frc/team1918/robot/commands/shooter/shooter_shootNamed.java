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
import frc.team1918.robot.subsystems.ShooterSubsystem.namedShots;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * A command that ...
 */
public class shooter_shootNamed extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"}) //Dont add "unused" under normal operation
  private final ShooterSubsystem m_shooter;
  private final namedShots m_name;

  /**
   * @param subsystem The subsystem used by this command.
   */
  public shooter_shootNamed(ShooterSubsystem subsystem, namedShots name) {
    m_shooter = subsystem;
    m_name = name;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Helpers.Debug.debug("Shooter: Named Shot - "+m_name);
    m_shooter.setShotName(m_name);
    switch (m_name) {
      case LOW:
        m_shooter.setShooterSpeed(Constants.Shooter.Shots.LOW.kSpeed);
        m_shooter.raiseHood(Constants.Shooter.Shots.LOW.kHood, Constants.Shooter.Shots.LOW.kHood2);
        m_shooter.startPreShooter();
        break;
      case BUMPER:
        m_shooter.setShooterSpeed(Constants.Shooter.Shots.BUMPER.kSpeed);
        m_shooter.raiseHood(Constants.Shooter.Shots.BUMPER.kHood, Constants.Shooter.Shots.BUMPER.kHood2);
        m_shooter.startPreShooter();
        break;
      case LINE:
        m_shooter.setShooterSpeed(Constants.Shooter.Shots.LINE.kSpeed);
        m_shooter.raiseHood(Constants.Shooter.Shots.LINE.kHood, Constants.Shooter.Shots.LINE.kHood2);
        m_shooter.startPreShooter();
        break;
      case NONE:
        m_shooter.stopShooter();
        m_shooter.raiseHood(false, false);
        m_shooter.stopPreShooter();
        break;
      case DEFAULT:
      default:
        m_shooter.setShooterSpeed(Constants.Shooter.Shots.DEFAULT.kSpeed);
        m_shooter.raiseHood(Constants.Shooter.Shots.DEFAULT.kHood, Constants.Shooter.Shots.DEFAULT.kHood2);
        m_shooter.startPreShooter();
        break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
