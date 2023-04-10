package frc.robot.subsystems.Hand;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.LoggyThings.LoggyWPI_TalonFX;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.Utils;

public class Wrist extends SubsystemBase{
    private static Wrist mInstance;

    public static Wrist getInstance() {
        if (mInstance == null) {
            mInstance = new Wrist();
        }
        return mInstance;
    }

    public static double currentAngle;
    
    private LoggyWPI_TalonFX mWristMotor;
    private CANCoder mWristCoder;
    private double correctedCanCoderPosition;

    private ArmFeedforward feedforward;

    public Wrist() {
        mWristMotor = new LoggyWPI_TalonFX(Constants.Hand.mWristMotor_ID);
        mWristCoder = new CANCoder(Constants.Hand.mWristCoder_ID);

        correctedCanCoderPosition = mWristCoder.getAbsolutePosition()
        - Constants.Hand.wristAngleSensor + Constants.Hand.wristAngleActual;
        correctedCanCoderPosition = Utils.clampDegreeMeasurement(correctedCanCoderPosition);
        mWristCoder.setPosition(correctedCanCoderPosition);

        mWristMotor.setInverted(false);

        feedforward = new ArmFeedforward(0, 0, 0, 0);
    }

    public double getWristAngle() {
        return mWristCoder.getPosition();
    }

    public void setWristPosition(double desiredAngle) {
        mWristMotor.set(ControlMode.Position, Conversions.degreesToFalcon(desiredAngle, Constants.Hand.WristGearRatio));
    }

    public void rawPower(double power) {
        mWristMotor.set(ControlMode.PercentOutput, power);
    }

    public void testBench() {
        // mIntakeMotor.set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward, feedforward.calculate(getWristAngle(), 0.5));
    }

    public void wristFeedFwd() {
        mWristMotor.set(ControlMode.PercentOutput, 0.1);
    }
}
