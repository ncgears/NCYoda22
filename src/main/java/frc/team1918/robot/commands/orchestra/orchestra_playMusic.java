package frc.team1918.robot.commands.orchestra;

import java.util.ArrayList;

import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj2.command.CommandBase;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.team1918.robot.Helpers;
import frc.team1918.robot.subsystems.OrchestraSubsystem;

public class orchestra_playMusic extends CommandBase { 
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"}) //Dont add "unused" under normal operation
  private final OrchestraSubsystem m_orchestra;

  public orchestra_playMusic(OrchestraSubsystem subsystem){
      m_orchestra = subsystem;

      addRequirements(subsystem);
  }
 // Called when the command is initially scheduled.
 @Override
 public void initialize() {
   Helpers.Debug.debug("Orchestra: Play");
   m_orchestra.LoadMusicSelection(0);
  
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
