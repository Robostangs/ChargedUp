package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Arm;

public class Lighting extends SubsystemBase {
    private static Lighting mLighting;
    
    private static Spark blinken;

    public static Lighting getInstance() {
        if (mLighting == null) {
            mLighting = new Lighting();
        }
        return mLighting;
    }
    
    public Lighting() {
        blinken = new Spark(Arm.blinkenPWM_ID);
    }


    public void setLights(double PWMVal) {
        blinken.set(PWMVal);
    }

    public static  double killLights() {
        return (0.99);
    }
}