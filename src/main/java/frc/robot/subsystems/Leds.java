package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Leds extends SubsystemBase {

    private static Leds mInstance;
    private Spark mBlinkin = new Spark(0);

    public static Leds getInstance() {
        if(mInstance == null) {
            mInstance = new Leds();
        }
        return mInstance;
    }

    public void setColor(double color) {
        mBlinkin.set(color);
    }
}