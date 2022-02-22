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
public class shooter_startShooterDefault extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterSubsystem m_shooter;
  private final boolean m_hoodup;

  /**
   * @param subsystem The subsystem used by this command.
   */
  public shooter_startShooterDefault(ShooterSubsystem subsystem, boolean hoodUp) {
    m_shooter = subsystem;
    m_hoodup = hoodUp;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setShooterSpeed(Constants.Shooter.kDefaultShooterSpeed);
    if (m_hoodup) {
      Helpers.Debug.debug("Shooter: Stop Shooter, Hood Up");
      m_shooter.startPreShooter();
      m_shooter.raiseHood(Constants.Air.stateHoodUp);
    } else {
      Helpers.Debug.debug("Shooter: Start Shooter, Hood Down");
      m_shooter.startPreShooter();
      m_shooter.raiseHood(!Constants.Air.stateHoodUp);
    }
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
    m_shooter.stopPreShooter();
    m_shooter.raiseHood(!Constants.Air.stateHoodUp);
    m_shooter.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
