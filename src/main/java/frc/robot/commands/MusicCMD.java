package frc.robot.commands;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.LoggyThings.LoggyWPI_TalonFX;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Music;

public class MusicCMD extends CommandBase {
    private final XboxController xDrive = new XboxController(OperatorConstants.kDriverControllerPort);

    private final Music mMusic;
    private TalonFX[] talonFXs;
    public static ArrayList<TalonFX> instruments = new ArrayList<TalonFX>();

    public MusicCMD(Music mMusic, TalonFX ... talons) {
            
        for(int i = 0; i < talons.length; i++) {
            instruments.add(talons[i]);
        }

        Music.init(instruments);
        this.mMusic = mMusic;
        addRequirements(mMusic);
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("Music Player", "Activated");
        mMusic.loadSong(Music.playlistOrder());
    }

//A for Play
//B for Pause
//Y for LoadSong

    @Override
    public void execute() {
        if (xDrive.getAButton()) {  
            mMusic.playSong();
        }
        if (xDrive.getBButton()) {
            mMusic.pauseSong();
        }
        if (xDrive.getYButton()) {
            mMusic.loadSong(Music.playlistOrder());
        }
        if (xDrive.getXButton()) {
        }
        mMusic.defaultCode();
    }

    public void end() {
        SmartDashboard.putString("Music Player", "Deactivated");
    }
}