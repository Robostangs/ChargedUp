package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Leds extends SubsystemBase {

    private static Leds mInstance;
    private Spark mBlinkin;

    public static Leds getInstance() {
        if(mInstance == null) {
            mInstance = new Leds();
        }
        return mInstance;
    }

    public void setColor(int color) {
        mBlinkin.set(color);
    }
}