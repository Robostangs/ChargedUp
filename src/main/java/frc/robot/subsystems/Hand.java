package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hand extends SubsystemBase {

    private static Hand mInstance;
    private static Compressor mCompressor;
    private static Solenoid mSolenoid;

    public enum HandState {
        OPEN, 
        CLOSED
    }

    public static Hand getInstance() {
        if(mInstance == null) {
            mInstance = new Hand();
        }
        return mInstance;
    }

    public Hand() {
        mCompressor = new Compressor(Constants.Hand.compressorID, PneumaticsModuleType.CTREPCM);
        mSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Hand.solenoidID);

        mCompressor.enableDigital();
    }

    public void setSolenoid(boolean state) {
        mSolenoid.set(state);
    }

    public boolean getSolenoid() {
        return mSolenoid.get();
    }
}