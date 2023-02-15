package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.music.Orchestra;


public class Music extends SubsystemBase {
    private static Music mMusic;
    private static Boolean Status;
    private static String Song;

    private static Orchestra mOrchestra;

    public static Music getInstance() {
        if(mMusic == null) {
            mMusic = new Music();
        }
        return mMusic;
    }

    public Music() {}

    public static void defaultCode() {
        SmartDashboard.putNumber("TimeStamp", mOrchestra.getCurrentTime());
        SmartDashboard.putString("Song", Song);
    }

    public static void playSong() {
        mOrchestra.play();
        Status = mOrchestra.isPlaying();
        SmartDashboard.putBoolean("Song Playing", Status);
    }

    public static void loadSong(String filePath) {
        mOrchestra.loadMusic(filePath);
        Song = filePath;
    }
    
}
