/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team1918.robot;

//Global imports
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
//Util imports
import frc.team1918.robot.utils.AndButton;
import frc.team1918.robot.utils.OrPOVButton;

//Subsystems imports
import frc.team1918.robot.subsystems.AutonSubsystem;
import frc.team1918.robot.subsystems.ClimberSubsystem;
import frc.team1918.robot.subsystems.CollectorSubsystem;
import frc.team1918.robot.subsystems.DriveSubsystem;
import frc.team1918.robot.subsystems.FeederSubsystem;
import frc.team1918.robot.subsystems.ShooterSubsystem;
//Commands imports
import frc.team1918.robot.commands.helpers.helpers_toggleDebug;
import frc.team1918.robot.commands.climber.*;
import frc.team1918.robot.commands.collector.*;
import frc.team1918.robot.commands.drive.*;
import frc.team1918.robot.commands.feeder.*;
import frc.team1918.robot.commands.shooter.*;
//CommandGroup imports
import frc.team1918.robot.commandgroups.cg_drive_initOdometry;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // private final PowerDistributionPanel m_pdp = new PowerDistributionPanel();
  private final Compressor m_air = new Compressor(PneumaticsModuleType.CTREPCM);
  //team 1918 subsystems
  // private final ClimberSubsystem m_climber = new ClimberSubsystem();
  private final CollectorSubsystem m_collector = new CollectorSubsystem();
  private final FeederSubsystem m_feeder = new FeederSubsystem();
  // private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final DriveSubsystem m_drive = new DriveSubsystem();
  //other subsystems
  private final AutonSubsystem m_auton = new AutonSubsystem();
  //team 1918 commands
  private final drive_resetGyro m_resetGyro = new drive_resetGyro(m_drive);
  private final cg_drive_initOdometry m_initOdom = new cg_drive_initOdometry(m_drive);
  // private final shooter_shootWall m_shooter_shootWall = new shooter_shootWall(shooter);
  // private final shooter_shootShort m_shooter_shootShort = new shooter_shootShort(shooter);
  // private final shooter_shootLine m_shooter_shootLine = new shooter_shootLine(shooter);
  // private final shooter_shootTrench m_shooter_shootTrench = new shooter_shootTrench(shooter);

  
  //Define the buttons and where they are bound
  //Driver Controller
  private Joystick dj = new Joystick(Constants.OI.OI_JOY_DRIVER);
  private JoystickButton btn_LOCKANGLE = new JoystickButton(dj, Constants.OI.Driver.BTN_LOCKANGLE);
  private JoystickButton btn_UNLOCKANGLE = new JoystickButton(dj, Constants.OI.Driver.BTN_UNLOCKANGLE);
  private JoystickButton btn_FEEDER_FWD = new JoystickButton(dj, Constants.OI.Driver.BTN_FEED_FWD);
  private JoystickButton btn_FEEDER_REV = new JoystickButton(dj, Constants.OI.Driver.BTN_FEED_REV);
  private JoystickButton btn_TOGGLE_DEBUG = new JoystickButton(dj, Constants.OI.Driver.BTN_TOG_DEBUG);
  private POVButton btn_GYRO_RESET = new POVButton(dj, Constants.OI.Driver.DPAD_GYRO_RESET);
  private POVButton btn_THROTUP_UP = new POVButton(dj, Constants.OI.Driver.DPAD_THROTUP_UP);
    private POVButton btn_THROTUP_UL = new POVButton(dj, Constants.OI.Driver.DPAD_THROTUP_UL);
    private POVButton btn_THROTUP_UR = new POVButton(dj, Constants.OI.Driver.DPAD_THROTUP_UR);
  private POVButton btn_THROTDN_DN = new POVButton(dj, Constants.OI.Driver.DPAD_THROTDN_DN);
    private POVButton btn_THROTDN_DL = new POVButton(dj, Constants.OI.Driver.DPAD_THROTDN_DL);
    private POVButton btn_THROTDN_DR = new POVButton(dj, Constants.OI.Driver.DPAD_THROTDN_DR);
  //OrPOVButtons are a custom button type to bind 3 DPAD directions to a single command. See utils/OrPOVButton
  private OrPOVButton orbtn_THROTUP = new OrPOVButton(btn_THROTUP_UP, btn_THROTUP_UL, btn_THROTUP_UR);
  private OrPOVButton orbtn_THROTDN = new OrPOVButton(btn_THROTDN_DN, btn_THROTDN_DL, btn_THROTDN_DR);

  //Operator Controller
  private Joystick oj = new Joystick(Constants.OI.OI_JOY_OPER);
  private JoystickButton btn_COLLECTOR_IN = new JoystickButton(oj, Constants.OI.Operator.BTN_COLLECTOR_IN);
  private POVButton btn_COLLECTOR_UP = new POVButton(oj, Constants.OI.Operator.DPAD_COLLECTOR_UP);
  private POVButton btn_COLLECTOR_DOWN = new POVButton(oj, Constants.OI.Operator.DPAD_COLLECTOR_DOWN);
  private JoystickButton btn_COLLECTOR_TOGGLE = new JoystickButton(oj, Constants.OI.Operator.BTN_TOG_MIDDOWN);

  //Special Bindings (AndButtons)
  // private JoystickButton btn_MECHZERO_KEY1 = new JoystickButton(dj, Constants.OI.Operator.BTN_MECHZERO);
  // private JoystickButton btn_MECHZERO_KEY2 = new JoystickButton(oj, Constants.OI.Operator.BTN_MECHZERO);
  // private AndButton andbtn_MECHZERO = new AndButton(btn_MECHZERO_KEY1,btn_MECHZERO_KEY2); //AndButton requires both to be true

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Enable closed loop control of compressor and enable it
    if(Constants.Air.isDisabled) m_air.disable();

    // Enable the camera server and start capture
    if(Constants.Global.CAMERA_ENABLED) {
      UsbCamera camera = CameraServer.startAutomaticCapture();
      camera.setResolution(640, 480);
    }

    // Set the default command that is run for the robot. Normally, this is the drive command
    m_drive.setDefaultCommand(
      new drive_defaultDrive(
        m_drive,
        () -> Helpers.OI.getAxisFwdValue(true),
        () -> Helpers.OI.getAxisStrafeValue(true),
        () -> Helpers.OI.getAxisTurnValue(true)
      )
    );
  }

  private void configureButtonBindings() {
    //The buttons here are named based on their functional purpose. This abstracts the purpose from which controller it is attached to.
    btn_GYRO_RESET.whenPressed(new drive_resetGyro(m_drive));
    btn_TOGGLE_DEBUG.whenPressed(new helpers_toggleDebug());
    // btn_SHOOT_WALL.whenPressed(new shooter_shootWall(m_shooter)).whenReleased(new shooter_stopShooter(m_shooter));
    // btn_SHOOT_LINE.whenPressed(new shooter_shootLine(m_shooter)).whenReleased(new shooter_stopShooter(m_shooter));
    // btn_SHOOT_SHORT.whenPressed(new shooter_shootShort(m_shooter)).whenReleased(new shooter_stopShooter(m_shooter));
    // btn_SHOOT_TRENCH.whenPressed(new shooter_shootTrench(m_shooter)).whenReleased(new shooter_stopShooter(m_shooter));
    btn_COLLECTOR_IN.whileHeld(new collector_intakeForward(m_collector)).whenReleased(new collector_intakeStop(m_collector));
    btn_COLLECTOR_DOWN.whenPressed(new collector_lowerIntake(m_collector));
    btn_COLLECTOR_UP.whenPressed(new collector_raiseIntake(m_collector));
    btn_COLLECTOR_TOGGLE.whenPressed(new collector_toggleIntake(m_collector));
    btn_LOCKANGLE.whenPressed(new drive_lockAngle(m_drive));
    btn_UNLOCKANGLE.whenPressed(new drive_unlockAngle(m_drive));

    //bind all 3 up and all 3 down for shooter throttle up/down
    // orbtn_THROTUP.whenPressed(new shooter_increaseThrottle(m_shooter));
    // orbtn_THROTDN.whenPressed(new shooter_decreaseThrottle(m_shooter));

    //bind both buttons requirement
    // andbtn_MECHZERO.whenPressed(new drive_moveAllToMechZero(m_drive));
  }

  // These functions return the commands, this is only needed for things that happen during robot init in Robot.java
  public drive_resetGyro getResetGyroCommand() {
    return m_resetGyro;
  }
  public cg_drive_initOdometry getInitOdomCommand() {
    return m_initOdom;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    //return m_autonCommand;
    return null;
  }
}
