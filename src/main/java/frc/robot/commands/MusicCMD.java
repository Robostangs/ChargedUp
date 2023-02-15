package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Music;

public class MusicCMD extends CommandBase {
    private Boolean CMD1;
    private String CMD2;
    private Boolean CMD3;

    public MusicCMD(Boolean Command1, String Command2, Boolean Command3, TalonFX ... talonFXs) {
        CMD1 = Command1;
        CMD2 = Command2;
        CMD3 = Command3;

            
        for(int i = 0; i < talonFXs.length; i++) {
            talonFXs[i].changeMotionControlFramePeriod(0);
        }
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("Music Player", "Activated");
    }

    @Override
    public void execute() {
        if (CMD1) {
            Music.playSong();
        }
        if (CMD2 != "null") {
            Music.loadSong(CMD2);
        }
        if (CMD3) {
            //Need CMD3
        }
        Music.defaultCode();
    }

    public void end() {
        SmartDashboard.putString("Music Player", "Deactivated");
    }
}
