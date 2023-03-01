package frc.robot.subsystems.Aesthetics;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.SwerveModule;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Swerve;

import java.io.File;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;

public class Music extends SubsystemBase {
    private static Music mMusic;
    private static Arm mArm;
    private static Swerve mSwerve;
    private static Orchestra mOrchestra = new Orchestra();
    private static Orchestra nOrchestra = new Orchestra();
    
    private static String Song;
    private static int instrumentNum;
    
    private boolean Status;
    
    private static int rawPlaylistLength;
    private static int SongNum;
    
    private final Command crossfade;
    
    public static final String[] playlist = {
        "MoveIt_MoveIt.chrp",
        "TwinkleStar.chrp",
        "HotCrossBuns.chrp"
    };
    
    public static int[] songLength = {
        29000, //Seconds
        29000
    };

    
    public Music() {
        mArm = Arm.getInstance();
        mSwerve = Swerve.getInstance();

        crossfade = new WaitCommand(1);
        
        instrumentNum = 0;
        rawPlaylistLength = (playlist.length - 1);
        SongNum = - 1;
    }    

    public static Music getInstance() {
        if (mMusic == null) {
            mMusic = new Music();
        }    
        return mMusic;
    }    
    
    public void defaultCode() {
        if (mOrchestra.getCurrentTime() >= songLength[SongNum]) {
            loadSong(playlistOrder());
            crossfade.schedule();
            playSong();
        }
        SmartDashboard.putNumber("TimeStamp", mOrchestra.getCurrentTime());
        SmartDashboard.putString("Song", Song);
    }

    public static void insertInstrument(TalonFX ... talons) {
        for (int i = 0; i < talons.length; i++) {
            if (instrumentNum <= 5) {
                    mOrchestra.addInstrument(talons[i]);
                    System.out.println("Added instrument to mOrchestra");
            } else {
                    nOrchestra.addInstrument(talons[i]);
                    System.out.println("Added instrument to nOrchestra");
            }
            instrumentNum++;
            System.out.println(instrumentNum);
        }
    }
    
    public void loadSong(String filename) {
        Song = filename;
        mOrchestra.loadMusic(File.separator + "home" + File.separator + "lvuser" + File.separator + "deploy" + File.separator + "Music" + File.separator + filename);
        nOrchestra.loadMusic(File.separator + "home" + File.separator + "lvuser" + File.separator + "deploy" + File.separator + "Music" + File.separator + filename);
    }

    public static String playlistOrder() {
        if (SongNum == (rawPlaylistLength)) {
            SongNum = 0;
        } else {
            SongNum++;
        }
        return playlist[SongNum];
    }

    public void playSong() {
        mOrchestra.play();
        nOrchestra.play();
    }
    
    public void pauseSong() {
        mOrchestra.pause();
        nOrchestra.pause();
    }
        
    public boolean playerStatus() {
        Status = mOrchestra.isPlaying();
        SmartDashboard.putBoolean("Song Playing", Status);
        return Status;
    }

    public void restartPlaylist() {
        SongNum = (rawPlaylistLength);
        loadSong(playlistOrder());
    }

    /** Disables all noise sources on robot
     * @see Compressor */
/*
     public void initialization() {
        mArm.mCompressor.disable();
    }
*/

    public void warningSound(double hertz) {
        mArm.setSound(hertz);
        for (SwerveModule mod : mSwerve.mSwerveMods) {
            mod.warningSound(hertz);
        }
    }
}
