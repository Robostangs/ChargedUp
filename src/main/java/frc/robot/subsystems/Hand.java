package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.LoggyThings.LoggyCANSparkMax;
import frc.robot.Constants;

public class Hand extends SubsystemBase {
    
    private static Hand mInstance;
    private static Compressor mCompressor;
    private static Solenoid mSolenoid;
    private static LoggyCANSparkMax mHandMotor;
    private SparkMaxPIDController pidController;

    public static Hand getInstance() {
        if(mInstance == null) {
            mInstance = new Hand();
        }
        return mInstance;
    }

    public Hand() {
        //TODO: Add compressor and solenoid auto in the default command
        mCompressor = new Compressor(Constants.Hand.compressorID, PneumaticsModuleType.REVPH);
        mSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.Hand.solenoidID);
        mHandMotor = new LoggyCANSparkMax(Constants.Hand.handMotorID, CANSparkMaxLowLevel.MotorType.kBrushless, "/Hand/");

        mHandMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        pidController = mHandMotor.getPIDController();
        pidController.setP(Constants.Hand.handMotorP);
        pidController.setI(Constants.Hand.handMotorI);
        pidController.setD(Constants.Hand.handMotorD);
    }

    public void setHandMotor(double speed) {
        mHandMotor.set(speed);
    }

    public void setArmMotorPosition(double position) {
        mHandMotor.set(position);
    }

    public double getHandPosition() {
        return mHandMotor.getEncoder().getPosition();
    }

    public void setSolenoid(boolean state) {
        mSolenoid.set(state);
    }

    public boolean getSolenoid() {
        return mSolenoid.get();
    }
}
