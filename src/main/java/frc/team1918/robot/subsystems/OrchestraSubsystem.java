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
    private TalonFX[] motors = { new TalonFX(Constants.Swerve.FL.DRIVE_MC_ID), new TalonFX(Constants.Swerve.FR.DRIVE_MC_ID), new TalonFX(Constants.Swerve.RL.DRIVE_MC_ID), new TalonFX(Constants.Swerve.FR.DRIVE_MC_ID), new TalonFX(Constants.Shooter.id_Motor2)}; //Instrument(motor) Array
    private String[] songs = new String[] {"Rickroll.chrp", "Megalovania.chrp", "Still-Alive.chrp", "Brawl-Theme.chrp"}; //Song Array
    /* When adding a new song, make sure the midi file has separate parts/instruments for each note.
       If it doesn't, the song won't play correctly */
    private int songSelection = 0; //Defaults to rickrolling everyone, as it should.
    private boolean orchestraPlaying = false;

    public void stopMusic(){
        orchestra.stop();
        orchestraPlaying = false;
        Helpers.Debug.debug("Orchestra: Stopped Playing");
    }

    public void enableOrchestra(){
        orchestraPlaying = true;
        Helpers.Debug.debug("Orchestra: Prepared to Play");
    }
    
    public void loadSong(int selection) { //Selects which song to load
        // if (selection < 0) {selection = 0;}
        // if (selection >= songs.length) {selection = songs.length;}
        selection = selection % songs.length; //make sure it always is within the bounds of the length
        orchestra.loadMusic(songs[selection]);
        Helpers.Debug.debug("Orchestra: Loaded Song: " +songs[selection]);
        new WaitCommand(0.5);
      }

    public void playMusic(){
        orchestra.play();
        Helpers.Debug.debug("Orchestra: Playing " +songs[songSelection]);
    }

    public void createOrchestra(){
        ArrayList<TalonFX> instruments = new ArrayList<TalonFX>();

        for (int i = 0; i < motors.length; ++i) {
          instruments.add(motors[i]);
        }
        new WaitCommand(1.0);
        orchestra = new Orchestra(instruments);
    }

    public void increaseSong(){
        songSelection++;
        songSelection = songSelection % songs.length;
        Helpers.Debug.debug("Orchestra: Song "+songs[songSelection]+" Selected");
    }

    public void decreaseSong(){
        songSelection--;
        songSelection = songSelection % songs.length;
        Helpers.Debug.debug("Orchestra: Song "+songs[songSelection]+" Selected");
    }
    public int getSong(){
        return songSelection;
    };
    public boolean getOrchestraPlaying(){
        return orchestraPlaying;
    }
}
