package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.LoggyThings.LoggyWPI_TalonFX;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.Utils;

public class Hand extends SubsystemBase{
    private static Hand mInstance;

    public static Hand getInstance() {
        if (mInstance == null) {
            mInstance = new Hand();
        }
        return mInstance;
    }

    public static boolean holding;
    public static double currentAngle;
    
    private LoggyWPI_TalonFX mWristMotor;
    private LoggyWPI_TalonFX mIntakeMotor;
    private CANCoder mWristCoder;
    private double correctedCanCoderPosition;

    private ArmFeedforward feedforward;

    public Hand() {
        holding = true;
        mWristMotor = new LoggyWPI_TalonFX(Constants.Hand.mWristMotor_ID);
        mIntakeMotor = new LoggyWPI_TalonFX(Constants.Hand.mIntakeMotor_ID);
        mWristCoder = new CANCoder(Constants.Hand.mWristCoder_ID);

        correctedCanCoderPosition = mWristCoder.getAbsolutePosition()
        - Constants.Hand.wristAngleSensor + Constants.Hand.wristAngleActual;
        correctedCanCoderPosition = Utils.clampDegreeMeasurement(correctedCanCoderPosition);
        mWristCoder.setPosition(correctedCanCoderPosition);

        mWristMotor.setInverted(false);
        mIntakeMotor.setInverted(false);

        feedforward = new ArmFeedforward(0, 0, 0, 0);
    }

    public boolean getHolding() {
        return holding;
    }

    public double getWristAngle() {
        return mWristCoder.getPosition();
    }

    public void Suck() {
        mIntakeMotor.set(ControlMode.PercentOutput, Constants.Hand.kSuckSpeed);
    }

    public void Spit() {
        mIntakeMotor.set(ControlMode.PercentOutput, Constants.Hand.kSpitSpeed);
    }

    public void setWristPosition(double desiredAngle) {
        mWristMotor.set(ControlMode.Position, Conversions.degreesToFalcon(desiredAngle, Constants.Hand.WristGearRatio));
    }

    public void rawPower(double power) {
        mWristMotor.set(ControlMode.PercentOutput, power);
    }

    public void testBench() {
        mIntakeMotor.set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward, feedforward.calculate(getWristAngle(), 0.5));
    }
}
