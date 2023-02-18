package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

import java.io.File;
import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;

public class Music extends SubsystemBase {
    private static Music mMusic;
    private static Orchestra mOrchestra;

    private static final Spark blinken = new Spark(0);
    
    private static Boolean Status;
    private static String Song = "Song";
    
    public static String[] playlist = {
        "TwinkleStar.chrp",
        "HotCrossBuns.chrp"
    };
    
    public static int[] songLength = {
        29000, //Seconds
        29000
    };

    public static int x = playlist.length;
    public static int playlistLength = playlist.length;
    public static int SongNum = playlistLength - 1;

    private static ArrayList<TalonFX> instruments = new ArrayList<TalonFX>();

    public static Music getInstance() {
        if (mMusic == null) {
            mMusic = new Music();
        }    
        return mMusic;
    }    
    
    public Music() {}    
    
    public static void init(ArrayList<TalonFX> instruments) {
        mOrchestra = new Orchestra(instruments);
    }

    public static void insertInstrument(TalonFX ... talons) {
        for (int i = 0; i < talons.length; i++) {
            mOrchestra.addInstrument(talons[i]);
        }
    }
    
    public void defaultCode() {
        if (mOrchestra.getCurrentTime() >= (songLength[SongNum] + 200)) {
            loadSong(playlistOrder());
            playSong();
        }
        SmartDashboard.putNumber("TimeStamp", mOrchestra.getCurrentTime());
        SmartDashboard.putString("Song", Song);
        lights();
    }

    public void playSong() {
        mOrchestra.play();
    }
    
    public void loadSong(String filename) {
        Song = filename;
        mOrchestra.loadMusic(File.separator + "home" + File.separator + "lvuser" + File.separator + "deploy" + File.separator + filename);
    }
    
    public void pauseSong() {
        mOrchestra.pause();
    }
        
    public static String playlistOrder() {
        if (SongNum == (playlistLength - 1)) {
            SongNum = 0;
        } else {
            SongNum++;
        }
        return playlist[SongNum];
    }
    
    public static void lights() {
        if (mOrchestra.isPlaying()) {
            blinken.set(0.65);
        } else {
            blinken.set(0.03);
        }
    }
    /*

    public static void skipSong() {
//        loadSong(playlistOrder());
        loadSong(playlist[1]);

}


    public static void playerStatus() {
        Status = mOrchestra.isPlaying();
        SmartDashboard.putBoolean("Song Playing", Status);
    }



    public static void restartPlaylist() {
        SongNum = (playlistLength - 1);
//        loadSong(playlistOrder());
    }
*/

}
