package frc.team1918.robot.commands.auton;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team1918.robot.Constants;
import frc.team1918.robot.subsystems.AutonSubsystem;
import frc.team1918.robot.subsystems.DriveSubsystem;

public class auton_pathDrive extends CommandBase {
  private final AutonSubsystem m_auton;
  private final DriveSubsystem m_drive;
  private HolonomicDriveController holonomicDriveController;
  private Trajectory trajectory;
  private Trajectory.State state;
  private ChassisSpeeds speeds = new ChassisSpeeds();
  private final Timer timer = new Timer();

  private Pose2d odometryPose = new Pose2d();

  /**
   * @param subsystem The subsystem used by this command.
   */
  public auton_pathDrive(AutonSubsystem subsystem, DriveSubsystem subsystem2) {
    m_auton = subsystem;
    m_drive = subsystem2;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    addRequirements(subsystem2);
  }

  public void initialize() {
    var p = 6.0;
    var d = p / 100.0;
    ProfiledPIDController thetaController =
        new ProfiledPIDController(
            2.5,
            0,
            0,
            new TrapezoidProfile.Constraints(Constants.Auton.kMaxOmega / 2.0, 3.14));
    thetaController.enableContinuousInput(Math.toRadians(-180), Math.toRadians(180));
    holonomicDriveController =
        new HolonomicDriveController(
            new PIDController(p, 0, d), new PIDController(p, 0, d), thetaController);

    holonomicDriveController.setEnabled(true);
    // DriveSubsystem.resetOdometry(trajectory.getInitialPose());
    // logger.info("Resetting timer!!!");
    // timer.reset();
  }

  @Override
  public void execute() {
    state = trajectory.sample(timer.get());
    odometryPose = m_drive.getPose();  //need to inject dependency
    speeds = holonomicDriveController.calculate(odometryPose, state, Rotation2d.fromDegrees(180));
    
    m_drive.drive( //need to inject dependency    
        speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false);
  }

}