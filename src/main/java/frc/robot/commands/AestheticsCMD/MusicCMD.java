package frc.robot.commands.AestheticsCMD;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Aesthetics.Music;

public class MusicCMD extends CommandBase {
    private final Music mMusic = Music.getInstance();
    private final Arm mArm = Arm.getInstance();

    public MusicCMD() {}

    @Override
    public void initialize() {
        mArm.mCompressor.disable();

        SmartDashboard.putBoolean("Music Player", mMusic.playerStatus());
        mMusic.loadSong(Music.playlistOrder());
    }

    @Override
    public void execute() {
        mMusic.defaultCode();
        
        System.out.println("Music Execution");

        if (mMusic.playerStatus()) {
            mMusic.pauseSong();
        } else {
            mMusic.playSong();
        }
    }

    public void end() {
        mArm.mCompressor.enableDigital();
        SmartDashboard.putString("Music Player", "Deactivated");
    }
}