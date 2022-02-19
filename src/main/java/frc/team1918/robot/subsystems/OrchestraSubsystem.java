package frc.team1918.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;
import frc.team1918.robot.Constants;
import frc.team1918.robot.Helpers;
import frc.team1918.robot.commands.helpers.helpers_debugMessage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1918.robot.subsystems.DriveSubsystem;

public class OrchestraSubsystem extends SubsystemBase {
    private Orchestra orchestra;
    private TalonFX[] motors = { new TalonFX(Constants.Swerve.FL.DRIVE_MC_ID), new TalonFX(Constants.Swerve.FR.DRIVE_MC_ID), new TalonFX(Constants.Swerve.RL.DRIVE_MC_ID), new TalonFX(Constants.Swerve.FR.DRIVE_MC_ID)}; //Instrument(motor) Array
    private String[] songs = new String[] { "Imperial-March.chrp", "Rickroll.chrp"}; //Song Array
    /* When adding a new song, make sure the midi file has separate parts for each note.
       If it doesn't, the song won't play correctly */
    private final DriveSubsystem m_drive = new DriveSubsystem();
    private int songSelection = 1;

    public void stopMusic(){
        orchestra.stop();
        m_drive.orchestraPlaying = false;
        Helpers.Debug.debug("Orchestra: Stopped Playing");
    }

    public void loadSong(int selection) { //Selects which song to load
        m_drive.orchestraPlaying = true;
        if (selection < 0) {selection = 0;}
        if (selection > songs.length) {selection = songs.length;}
        orchestra.loadMusic(songs[selection]);
        Helpers.Debug.debug("Orchestra: Loaded Song " +selection);
      }

    public void playMusic(){
        orchestra.play();
        Helpers.Debug.debug("Orchestra: Playing Song");
    }

    public void createOrchestra(){
        ArrayList<TalonFX> instruments = new ArrayList<TalonFX>();

        for (int i = 0; i < motors.length; ++i) {
          instruments.add(motors[i]);
        }
    
        orchestra = new Orchestra(instruments);
    }

    public void increaseSong(){
        songSelection++;
        Helpers.Debug.debug("Orchestra: Song "+songSelection+" Selected");
    }

    public void decreaseSong(){
        songSelection--;
        Helpers.Debug.debug("Orchestra: Song "+songSelection+" Selected");
    }
    public int getSong(){
        return songSelection;
    };
}
