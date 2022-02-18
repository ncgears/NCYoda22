package frc.team1918.robot.commands.orchestra;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team1918.robot.Helpers;

import com.ctre.phoenix.music.Orchestra;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import java.util.ArrayList;

public class orchestra_loadAndPlay extends CommandBase {
  
    Orchestra orchestra;
    TalonFX[] motors = { new TalonFX(31), new TalonFX(32), new TalonFX(33), new TalonFX(35)}; 
    String[] songs = new String[] { "Imperial-March.chrp", "Rickroll.chrp", "Rickroll2.chrp", "Rickroll3.chrp"};

    private void loadSong(int selection) {
      orchestra.loadMusic(songs[selection]);
      Helpers.Debug.debug("Orchestra: Loaded Music");
    }

  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ArrayList<TalonFX> instruments = new ArrayList<TalonFX>();

    for (int i = 0; i < motors.length; ++i) {
      instruments.add(motors[i]);
    }

    orchestra = new Orchestra(instruments);

    loadSong(0);

  }
 

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      orchestra.play();
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
