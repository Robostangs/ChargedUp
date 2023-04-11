package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utils;

public class RollyHand extends SubsystemBase {
    private static RollyHand instance;

    private TalonFX mRotator;
    private TalonFX mRoller;
    private CANCoder mRotatorCanCoder;
    private double mCanCoderOffset;

    private double feedForward;

    public static RollyHand getInstance() {
        if (instance == null) {
            instance = new RollyHand();
        }
        return instance;
    }

    public RollyHand() {
        feedForward = 0;
        mRotator = new TalonFX(Constants.Arm.RollyHandRotatorID);
        mRoller = new TalonFX(Constants.Arm.RollyHandRollerID);
        mRotatorCanCoder = new CANCoder(Constants.Arm.RollyHandRotatorCanCoderID);

        
        double correctedRotatorCanCoderPostion = mRotatorCanCoder.getAbsolutePosition()
            - Constants.Arm.rotatorAngleSensor + Constants.Arm.rotatorAngleActualDifference;
        correctedRotatorCanCoderPostion = Utils.clampDegreeMeasurement(correctedRotatorCanCoderPostion);

        mRotatorCanCoder.setPosition(correctedRotatorCanCoderPostion);
        mRotator.config_kP(0, Constants.Arm.RollyHandRotatorP);
        mRotator.config_kI(0, Constants.Arm.RollyHandRotatorI);
        mRotator.config_kD(0, Constants.Arm.RollyHandRotatorD);
    }

    private void addRequirements(RollyHand instance2) {
    }

    public void setRollerPower(double power) {
        mRoller.set(ControlMode.PercentOutput, power);
    }

    public void setRotatorFeedForward(double power){
        feedForward = power;
    }

    public void setRotatorAngle(double angle) {
        mRotator.set(ControlMode.Position, (angle / 360) * 4096, DemandType.ArbitraryFeedForward, feedForward);
    }
}
