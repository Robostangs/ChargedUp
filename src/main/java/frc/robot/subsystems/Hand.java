package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.LoggyThings.LoggyWPI_TalonFX;
import frc.lib.math.Conversions;
import frc.robot.Constants;

public class Hand extends SubsystemBase{
    private static Hand mInstance;

    public static Hand getInstance() {
        if (mInstance == null) {
            mInstance = new Hand();
        }
        return mInstance;
    }

    public boolean holding;
    public double currentAngle;
    
    private LoggyWPI_TalonFX mWristMotor;
    private LoggyWPI_TalonFX mIntakeMotor;

    private ArmFeedforward feedforward = new ArmFeedforward(0, 0, 0, 0);

    public Hand() {
        holding = true;
        mWristMotor = new LoggyWPI_TalonFX(Constants.Hand.mWristMotor);
        mIntakeMotor = new LoggyWPI_TalonFX(Constants.Hand.mIntakeMotor);

        mWristMotor.setInverted(false);
        mIntakeMotor.setInverted(false);
    }

    public boolean getHolding() {
        return holding;
    }

    public void Suck() {
        mIntakeMotor.set(ControlMode.PercentOutput, Constants.Hand.kSuckSpeed);
    }

    public void Spit() {
        mIntakeMotor.set(ControlMode.PercentOutput, Constants.Hand.kSpitSpeed);
    }

    public void rotateWrist(DoubleSupplier rotation) {
        double desiredAngle = currentAngle + rotation.getAsDouble();
        mWristMotor.set(ControlMode.Position, Conversions.degreesToFalcon(0, 0));
    }
}
