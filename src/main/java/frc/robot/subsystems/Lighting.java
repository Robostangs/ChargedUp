package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Arm;

public class Lighting extends SubsystemBase {
    private static Lighting mLighting;
    
    private Spark blinken;

    public double coneLight = 0.65;
    public double cubeLight = 0.91;
    public double killLights = 0.99;

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

    public void lightsOff() {
        blinken.set(0.99);
    }
}