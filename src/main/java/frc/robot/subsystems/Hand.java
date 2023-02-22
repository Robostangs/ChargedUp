package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hand extends SubsystemBase {

    private static Hand mInstance;
    private DoubleSolenoid mSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6,7);


    public enum HandState {
        OPEN, 
        CLOSED,
        TOGGLE,
    }

    public enum HandHolding {
        CUBE,
        CONE
    }

    public static Hand getInstance() {
        if(mInstance == null) {
            mInstance = new Hand();
        }
        return mInstance;
    }

    public Hand() {
        // mSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Hand.mHandSolenoidFwd, Constants.Hand.mHandSolenoidRev);
    }

    public void setSolenoid(boolean state) {
        // System.out.println("here");
        if(state) {
            mSolenoid.set(DoubleSolenoid.Value.kForward);
        } else {
            mSolenoid.set(DoubleSolenoid.Value.kReverse);
        }
    }

    public boolean getSolenoid() {
        return mSolenoid.get() == DoubleSolenoid.Value.kForward;
    }
}