package frc.team1918.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OrchestraSubsystem extends SubsystemBase{
     /* The orchestra object that holds all the instruments */
     Orchestra _orchestra;

     /* Talon FXs to play music through.  
     More complex music MIDIs will contain several tracks, requiring multiple instruments.  */
     WPI_TalonFX [] _fxes =  { new WPI_TalonFX(31, "rio"), new WPI_TalonFX(32, "rio"), new WPI_TalonFX(33, "rio"), new WPI_TalonFX(34, "rio") };
 
     /* An array of songs that are available to be played*/
   String[] _songs = new String[] {
     "Imperial-March.chrp",
     
   };
 
     /* track which song is selected for play */
     int _songSelection = 0;
 
     /* overlapped actions */
     int _timeToPlayLoops = 0;
 
     public void LoadMusicSelection(int offset)
     {
         /* increment song selection */
         _songSelection += offset;
         /* wrap song index in case it exceeds boundary */
         if (_songSelection >= _songs.length) {
             _songSelection = 0;
         }
         if (_songSelection < 0) {
             _songSelection = _songs.length - 1;
         }
         /* load the chirp file */
         _orchestra.loadMusic(_songs[_songSelection]); 
 
         /* print to console */
         System.out.println("Song selected is: " + _songs[_songSelection]);
         
         /* schedule a play request, after a delay.  
             This gives the Orchestra service time to parse chirp file.
             If play() is called immedietely after, you may get an invalid action error code. */
         _timeToPlayLoops = 10;
     }
}
