/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team1918.robot;

import edu.wpi.first.util.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import frc.team1918.robot.subsystems.ClimberSubsystem;
// import frc.team1918.robot.subsystems.CollectorSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public Alliance m_alliance;
  private Command m_autonomousCommand;
  private Command m_disableCommand;
  // private Command m_initOdom;
  // private Command m_resetGyro;
  private SendableChooser<Command> m_chooser = new SendableChooser<>();

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    //Disable LiveWindow
    LiveWindow.disableAllTelemetry();
    //Create forwarder for photonvision
    PortForwarder.add(5800,"photonvision.local",5800);
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    if(Constants.Auton.isDisabled) {
      m_chooser.setDefaultOption("Auton Disabled", m_robotContainer.getRobotCommand("auton_disabled"));
    } else {
      m_chooser.setDefaultOption("AR1 3 Ball", m_robotContainer.getRobotCommand("auton_ar1ThreeBall"));
      m_chooser.addOption("AR2 2 Ball", m_robotContainer.getRobotCommand("auton_ar2TwoBall"));
      m_chooser.addOption("AR3 4 Ball", m_robotContainer.getRobotCommand("auton_ar3FourBall"));
      m_chooser.addOption("AR4 4 Ball #2", m_robotContainer.getRobotCommand("auton_ar4FourBall2"));
      m_chooser.addOption("AC1 1 Ball", m_robotContainer.getRobotCommand("auton_ac1OneBall"));
      m_chooser.addOption("AL1 2 Ball", m_robotContainer.getRobotCommand("auton_al1TwoBall"));
      // m_chooser.addOption("4 Ball Auto", m_robotContainer.getRobotCommand("auton_4BallAuto"));
      m_chooser.addOption("Basic Drive", m_robotContainer.getRobotCommand("auton_BasicDriveAuto"));
      m_chooser.addOption("Basic Shooting", m_robotContainer.getRobotCommand("auton_BasicShootingAuto"));
    }
    SmartDashboard.putData(m_chooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    m_disableCommand = m_robotContainer.getRobotCommand("resetRobot");
    if (m_disableCommand != null) m_disableCommand.schedule();
    // m_dc1 = m_robotContainer.getDisableCommand(1);
    // if (m_dc1 != null) m_dc1.schedule();
    // m_dc2 = m_robotContainer.getDisableCommand(2);
    // if (m_dc2 != null) m_dc2.schedule();
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_alliance = DriverStation.getAlliance(); //Put Alliance.Red or Alliance.Blue in Robot.m_alliance
    // m_resetGyro = m_robotContainer.getResetGyroCommand();
    // if (m_resetGyro != null) m_resetGyro.schedule();

    // m_initOdom = m_robotContainer.getInitOdomCommand(); 
    // if (m_initOdom != null) m_initOdom.schedule();

    // Helpers.Debug.debug("Getting Auton Command for "+Constants.Auton.autonToRun);
    // m_autonomousCommand = m_robotContainer.getRobotCommand(Constants.Auton.autonToRun);
    m_autonomousCommand = m_chooser.getSelected();
    if (m_autonomousCommand != null && !Constants.Auton.isDisabled) m_autonomousCommand.schedule();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) m_autonomousCommand.cancel();

    // m_resetGyro = m_robotContainer.getResetGyroCommand();
    // if (m_resetGyro != null) m_resetGyro.schedule();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

}
