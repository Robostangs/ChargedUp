package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Arm;

public class Lighting extends SubsystemBase {
    private static Lighting mInstance;
    
    private static Spark blinken;

    public static Lighting getInstance() {
        if (mInstance == null) {
            mInstance = new Lighting();
        }
        return mInstance;
    }
    
    public Lighting() {
        blinken = new Spark(Arm.blinkenPWM_ID);
        setLights(killLights());
    }


    public void setLights(double PWMVal) {
        blinken.set(PWMVal);
    }

    public double killLights() {
        return (0.99);
    }
}