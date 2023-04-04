package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Arm;

public class Lighting extends SubsystemBase {
    private static Lighting mInstance;
    
    private static Spark blinken;

    public static double lastLight;

    public static final double kConeStatic = 0.1;
    public static final double kConeBlink = 0.15;
    public static final double kCubeStatic = 0.2;
    public static final double kCubeBlink = 0.25;
    public static final double kKillLights = 0.99;

    public static Lighting getInstance() {
        if (mInstance == null) {
            mInstance = new Lighting();
        }
        return mInstance;
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

    public double getPWM() {
        return blinken.get();
    }
}