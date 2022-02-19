package frc.team1918.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OrchestraSubsystem extends SubsystemBase {
    public Orchestra orchestra;
    public TalonFX[] motors = { new TalonFX(31), new TalonFX(32), new TalonFX(33), new TalonFX(35)}; //Instrument(motor) Array
    public String[] songs = new String[] { "Imperial-March.chrp", "Rickroll.chrp"}; //Song Array
    /* When adding a new song, make sure the midi file has separate parts for each note.
       If it doesn't, the song won't play correctly */
}
