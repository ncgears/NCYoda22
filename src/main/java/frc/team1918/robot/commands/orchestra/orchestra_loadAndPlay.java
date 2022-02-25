package frc.team1918.robot.commands.orchestra;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team1918.robot.Helpers;

import com.ctre.phoenix.music.Orchestra;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import java.util.ArrayList;

import frc.team1918.robot.subsystems.OrchestraSubsystem;

public class orchestra_loadAndPlay extends CommandBase {
  private final OrchestraSubsystem m_orchestra;
  private int m_song;
  
  public orchestra_loadAndPlay(OrchestraSubsystem subsystem){
    m_orchestra = subsystem;
    // m_song = selection;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_song = m_orchestra.getSong();
    m_orchestra.createOrchestra();
    m_orchestra.loadSong(m_song); //Change to select song
    m_orchestra.playMusic();
  }
 
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }    
}
