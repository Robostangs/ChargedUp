package frc.robot.subsystems.Aesthetics;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lighting extends SubsystemBase {
    private static Lighting mLighting;
    
    private final Spark blinken;

    public static Lighting getInstance() {
        if (mLighting == null) {
            mLighting = new Lighting();
        }
        return mLighting;
    }

    public Lighting() {
        blinken = new Spark(0);
    }
    public void setLights(double PWMVal) {
        blinken.set(PWMVal);
    }
}