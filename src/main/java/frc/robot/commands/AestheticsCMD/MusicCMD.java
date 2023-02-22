package frc.robot.commands.AestheticsCMD;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Aesthetics.Music;

public class MusicCMD extends CommandBase {
    private final Music mMusic = Music.getInstance();

    public MusicCMD() {}

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("Music Player", mMusic.playerStatus());
        mMusic.loadSong(Music.playlistOrder());
    }

    @Override
    public void execute() {
        mMusic.loadSong(Music.playlistOrder());
        mMusic.defaultCode();

        if (mMusic.playerStatus()) {
            mMusic.pauseSong();
        } else {
            mMusic.playSong();
        }
    }

    public void end() {
        SmartDashboard.putString("Music Player", "Deactivated");
    }
}