package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hand extends SubsystemBase {

    private static Hand mInstance;
    private DoubleSolenoid mSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Hand.mHandSolenoidFwd,Constants.Hand.mHandSolenoidRev);
    public boolean holdingCone = true;
    public boolean gripping = false;

    public static Hand getInstance() {
        if(mInstance == null) {
            mInstance = new Hand();
        }
        return mInstance;
    }

    public void periodic() {
        if(gripping) {
            mSolenoid.set(DoubleSolenoid.Value.kForward);
        } else {
            mSolenoid.set(DoubleSolenoid.Value.kReverse);
        }
        SmartDashboard.putBoolean("/Hand/Gripping", gripping);
        SmartDashboard.putBoolean("/Hand/HoldingCone", holdingCone);
        
        if(holdingCone) {
        } else {
        }

        
    }

    public void setGripping(boolean g) {
        gripping = g;
    }

    public boolean getGripping() {
        return mSolenoid.get() == DoubleSolenoid.Value.kForward;
    }

    public void setHolding(boolean h) {
        holdingCone = h;
    }

    public boolean getHolding() {
        return holdingCone;
    }
}