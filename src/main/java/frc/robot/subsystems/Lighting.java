package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Lights;

public class Lighting extends SubsystemBase {
    private static Lighting mLighting;
    
    private Spark blinkin;

    public static double lastLight;

    public static Lighting getInstance() {
        if (mLighting == null) {
            mLighting = new Lighting();
        }
        return mLighting;
    }
    
    public Lighting() {
        blinkin = new Spark(Lights.blinkinPWM_ID);
    }


    public void setLights(double PWMVal) {
        blinkin.set(PWMVal);
    }

    public void lightsOff() {
        blinkin.set(Lights.kKillLights);
    }

    public double getPWM() {
        return blinkin.get();
    }
}