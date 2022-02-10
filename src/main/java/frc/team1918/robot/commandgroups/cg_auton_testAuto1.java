/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team1918.robot.commandgroups;

// import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team1918.paths.*;
import frc.team1918.robot.commands.collector.*;
import frc.team1918.robot.commands.drive.drive_followTrajectory;
import frc.team1918.robot.commands.shooter.*;
import frc.team1918.robot.commands.feeder.*;
import frc.team1918.robot.subsystems.CollectorSubsystem;
import frc.team1918.robot.subsystems.DriveSubsystem;
import frc.team1918.robot.subsystems.FeederSubsystem;
import frc.team1918.robot.subsystems.ShooterSubsystem;

public class cg_auton_testAuto1 extends SequentialCommandGroup {
  private final CollectorSubsystem m_collector;
  private final DriveSubsystem m_drive;
  private final FeederSubsystem m_feeder;
  private final ShooterSubsystem m_shooter;

  public cg_auton_testAuto1(DriveSubsystem drive, CollectorSubsystem collector, FeederSubsystem feeder, ShooterSubsystem shooter) {
    m_collector = collector;
    m_drive = drive;
    m_feeder = feeder;
    m_shooter = shooter;
    addRequirements(m_collector, m_drive, m_feeder, m_shooter);

    addCommands(
        //this is a comma separated list of commands, thus, the last one should not have a comma
        new collector_deployIntake(m_collector), //deploy collector
        new ParallelDeadlineGroup( //do until trajectory complete
          // new drive_followTrajectory(m_drive, new auton_getPath("GetBall2")), //go to ball 2 location
          new drive_followTrajectory(m_drive, new OnePointEightMetersForward()),
          // new WaitCommand(3.0), //placeholder for trajectory follower
          new collector_intakeForward(m_collector) //start collector
        ),
        new ParallelCommandGroup(
          new collector_intakeStop(m_collector), //stop collector
          new collector_retractIntake(m_collector)  //retract collector
          // new drive_followTrajectory(m_drive, new auton_getPath("ShootingPosition1")), //go to shooting position from ball 2
        ),
        new shooter_shootNamed(m_shooter, "AutonTarmac"),
        new WaitCommand(1.0), //wait for shooter to be at speed
        new feeder_advance(m_feeder), //start advancing the feeder
        new WaitCommand(1.25), //give time for shot
        new ParallelCommandGroup(
          new shooter_stopShooter(m_shooter), //stop shooter
          new shooter_hoodDown(m_shooter), //lower the hood
          new feeder_stop(m_feeder) //stop the feeder
        ),
        new collector_deployIntake(m_collector), //deploy collector
        new ParallelDeadlineGroup( //do until trajectory complete
          // new drive_followTrajectory(m_drive, new auton_getPath("GetBall3And4")), //go to ball 3 and 4 location
          new drive_followTrajectory(m_drive, new CollectSecondBall()),
          // new WaitCommand(3.0), //placeholder for trajectory follower
          new collector_intakeForward(m_collector) //start collector
        ),
        new drive_followTrajectory(m_drive, new GoHome()),
        new shooter_shootNamed(m_shooter, "AutonTarmac"), //start the shooter for named shot
        new feeder_advance(m_feeder), //start advancing the feeder
        new WaitCommand(1.25), //give time for shot
        new ParallelCommandGroup(
          new shooter_stopShooter(m_shooter), //stop shooter
          new shooter_hoodDown(m_shooter), //lower the hood
          new feeder_stop(m_feeder) //stop the feeder
        )
    );
  }
}