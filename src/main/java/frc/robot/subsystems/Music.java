package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj.Timer;

public class Music {

    private static Music instance;

    private final Orchestra mOrchestra = new Orchestra();

    public static Music getInstance() {
        if (instance == null) {
            instance = new Music();
        }
        return instance;
    }

    public void addMotors(TalonFX... Falcons) {
        for (TalonFX Falcon : Falcons) {
            mOrchestra.addInstrument(Falcon);
        }
    }

    public void loadMusic(String musicFilePath) {
        // Music File Path Just has to be the name of the chirp file in deploy, like
        // dynamite.chrp
        mOrchestra.loadMusic(musicFilePath);
        Timer.delay(2);
        play();
    }

    private void play() {
        mOrchestra.play();
    }
}
