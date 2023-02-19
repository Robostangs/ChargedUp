package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hand extends SubsystemBase {

    private static Hand mInstance;
    private static Solenoid mSolenoid;

    public enum HandState {
        OPEN, 
        CLOSED,
        TOGGLE,
    }

    public static Hand getInstance() {
        if(mInstance == null) {
            mInstance = new Hand();
        }
        return mInstance;
    }

    public Hand() {
        mSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Hand.mHandSolenoid);
    }

    public void setSolenoid(boolean state) {
        mSolenoid.set(state);
    }

    public boolean getSolenoid() {
        return mSolenoid.get();
    }
}