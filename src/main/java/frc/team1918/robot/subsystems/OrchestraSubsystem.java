package frc.team1918.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;
import frc.team1918.robot.Constants;
import frc.team1918.robot.Helpers;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class OrchestraSubsystem extends SubsystemBase {
    private Orchestra orchestra;
    private TalonFX[] motors = { new TalonFX(Constants.Swerve.FL.DRIVE_MC_ID), new TalonFX(Constants.Swerve.FR.DRIVE_MC_ID), new TalonFX(Constants.Swerve.RL.DRIVE_MC_ID), new TalonFX(Constants.Swerve.FR.DRIVE_MC_ID)}; //Instrument(motor) Array
    private String[] songs = new String[] {"Rickroll.chrp", "Megalovania.chrp", "Still-Alive.chrp", "Brawl-Theme.chrp"}; //Song Array
    /* When adding a new song, make sure the midi file has separate parts for each note.
       If it doesn't, the song won't play correctly */
    private int songSelection = 0;
    private boolean orchestraPlaying = false;

    public void stopMusic(){
        orchestra.stop();
        orchestraPlaying = false;
        Helpers.Debug.debug("Orchestra: Stopped Playing");
    }

    public void loadSong(int selection) { //Selects which song to load
        orchestraPlaying = true;
        if (selection < 0) {selection = 0;}
        if (selection > songs.length) {selection = songs.length;}
        orchestra.loadMusic(songs[selection]);
        Helpers.Debug.debug("Orchestra: Loaded Song: " +songs[selection]);
        new WaitCommand(1.0);
      }

    public void playMusic(){
        orchestra.play();
        Helpers.Debug.debug("Orchestra: Playing Song");
    }

    public void createOrchestra(){
        orchestraPlaying = true;
        ArrayList<TalonFX> instruments = new ArrayList<TalonFX>();

        for (int i = 0; i < motors.length; ++i) {
          instruments.add(motors[i]);
        }
        new WaitCommand(1.0);
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
    public boolean getOrchestraPlaying(){
        return orchestraPlaying;
    }
}
