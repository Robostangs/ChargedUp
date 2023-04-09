package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Lights;
import frc.robot.Constants.Lights.ConeCube;

public class Lighting extends SubsystemBase {
    private static Lighting mLighting;
    
    private static Spark blinkin;

    public static Constants.Lights.ConeCube lastLight;

    public static Lighting getInstance() {
        if (mLighting == null) {
            mLighting = new Lighting();
        }
        return mLighting;
    }
    
    public Lighting() {
        blinkin = new Spark(Lights.blinkinPWM_ID);
        lastLight = ConeCube.kCube;
        Constants.Lights.prevLightReqCMD = null;
    }

    public void setLights(double PWMVal) {
        blinkin.set(PWMVal);
    }

    public void lightsOff() {
        blinkin.set(Lights.kKillLights);
    }

    public static double getPWM() {
        return blinkin.get();
    }
}