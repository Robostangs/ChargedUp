package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lighting extends SubsystemBase {
    private static Lighting mLighting;
    
    private final Spark blinken1;
    private final Spark blinken2;

    public static Lighting getInstance() {
        if (mLighting == null) {
            mLighting = new Lighting();
        }
        return mLighting;
    }

    public Lighting() {
        blinken2 = new Spark(1);
        blinken1 = new Spark(0);
    }

    public void setLights(double PWMVal) {
        blinken1.set(PWMVal);
        blinken2.set(PWMVal);
    }

    
}
