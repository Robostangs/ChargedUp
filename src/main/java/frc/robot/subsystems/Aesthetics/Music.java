package frc.robot.subsystems.Aesthetics;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.File;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;

public class Music extends SubsystemBase {
    private static Music mMusic;
    private static Orchestra mOrchestra = new Orchestra();
    private static Orchestra nOrchestra = new Orchestra();
    
    private static String Song;
    private static int instrumentNum = 0;
    
    private static boolean Status;
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

    private final int crossfade = 1000;

    public static Music getInstance() {
        if (mMusic == null) {
            mMusic = new Music();
        }    
        return mMusic;
    }    
    
    public Music() {
    }    

    public static void insertInstrument(TalonFX ... talons) {
        for (int i = 0; i < talons.length; i++) {
            if (instrumentNum <= 5) {
                    mOrchestra.addInstrument(talons[i]);
            } else {
                    nOrchestra.addInstrument(talons[i]);
            }
            instrumentNum++;
        }
    }
    
    public void defaultCode() {
        if (mOrchestra.getCurrentTime() >= (songLength[SongNum] + crossfade)) {
            loadSong(playlistOrder());
            playSong();
        }
        SmartDashboard.putNumber("TimeStamp", mOrchestra.getCurrentTime());
        SmartDashboard.putString("Song", Song);
    }

    public void playSong() {
        mOrchestra.play();
        nOrchestra.play();
    }
    
    public void loadSong(String filename) {
        Song = filename;
        mOrchestra.loadMusic(File.separator + "home" + File.separator + "lvuser" + File.separator + "deploy" + File.separator + filename);
        nOrchestra.loadMusic(File.separator + "home" + File.separator + "lvuser" + File.separator + "deploy" + File.separator + filename);
    }
    
    public void pauseSong() {
        mOrchestra.pause();
        nOrchestra.pause();
    }
        
    public static String playlistOrder() {
        if (SongNum == (playlistLength - 1)) {
            SongNum = 0;
        } else {
            SongNum++;
        }
        return playlist[SongNum];
    }
    
    public void skipSong() {
        loadSong(playlistOrder());
    }

    public boolean playerStatus() {
        Status = mOrchestra.isPlaying();
        SmartDashboard.putBoolean("Song Playing", Status);
        return Status;
    }

    public void restartPlaylist() {
        SongNum = (playlistLength - 1);
        loadSong(playlistOrder());
    }
}