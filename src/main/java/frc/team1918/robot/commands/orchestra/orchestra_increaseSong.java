package frc.team1918.robot.commands.orchestra;

import edu.wpi.first.wpilibj2.command.CommandBase;
import com.ctre.phoenix.music.Orchestra;

import frc.team1918.robot.Helpers;
import frc.team1918.robot.subsystems.OrchestraSubsystem;

public class orchestra_increaseSong extends CommandBase {
    private final OrchestraSubsystem m_orchestra;

    public orchestra_increaseSong(OrchestraSubsystem subsystem){
        m_orchestra = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
      m_orchestra.increaseSong();
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

