package frc.robot.subsystems;

import java.lang.Math;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.LoggyThings.LoggyCANSparkMax;
import frc.robot.Constants;
import frc.robot.Utils;

public class Arm extends SubsystemBase{
    private static Arm mInstance;
    private LoggyCANSparkMax mShoulderMotor;
    private LoggyCANSparkMax mElbowMotor;
    private SparkMaxPIDController mShoulderPidController;
    private SparkMaxPIDController mElbowPidController;

    private Utils.Vector2D mArmPosition;

    public static Arm getInstance() {
        if(mInstance == null) {
            mInstance = new Arm();
        }
        return mInstance;
    }

    public Arm() {
        mShoulderMotor = new LoggyCANSparkMax(Constants.Arm.shoulderMotorID, CANSparkMaxLowLevel.MotorType.kBrushless, "Shoulder");
        mElbowMotor = new LoggyCANSparkMax(Constants.Arm.shoulderMotorID, CANSparkMaxLowLevel.MotorType.kBrushless, "Elbow");

        mShoulderPidController = mShoulderMotor.getPIDController();
        mElbowPidController = mElbowMotor.getPIDController();

        mShoulderPidController.setP(Constants.Arm.shoulderMotorP);
        mShoulderPidController.setI(Constants.Arm.shoulderMotorI);
        mShoulderPidController.setD(Constants.Arm.shoulderMotorD);
        mShoulderPidController.setIZone(Constants.Arm.shoulderMotorI, 0);

        mElbowPidController.setP(Constants.Arm.elbowMotorP);
        mElbowPidController.setI(Constants.Arm.elbowMotorI);
        mElbowPidController.setD(Constants.Arm.elbowMotorD);
        mElbowPidController.setIZone(Constants.Arm.elbowMotorI, 0);

        mShoulderMotor.setIdleMode(IdleMode.kBrake);
        mShoulderMotor.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Arm Position X", this.mArmPosition::getX, this.mArmPosition::setX);
        builder.addDoubleProperty("Arm Position Y", this.mArmPosition::getY, this.mArmPosition::setY);
        builder.addDoubleProperty( "Shoulder Rotation", this::getShoulderRotation, null);
        builder.addDoubleProperty( "Elbow Rotation", this::getElbowRotation, null);
    }


    public Utils.Vector2D calculateArmPosition(Utils.Vector2D targetPos) {
        double q2 = Math.acos(Math.pow(targetPos.x,2) + Math.pow(targetPos.y,2) - Math.pow(Constants.Arm.upperarmLength, 2) - Math.pow(Constants.Arm.forearmLength, 2));
        double q1 = Math.atan2(targetPos.y, targetPos.x) - Math.atan2(Constants.Arm.forearmLength * Math.sin(q2), Constants.Arm.forearmLength + Constants.Arm.upperarmLength*Math.cos(q2));
        return new Utils.Vector2D(q1, q1);
    }

    public Utils.Vector2D getmArmPosition() {
        return mArmPosition;
    }
    
    
    public double getShoulderRotation() {
        return mShoulderMotor.getEncoder().getPosition();
    }
    
    public double getElbowRotation() {
        return mElbowMotor.getEncoder().getPosition();
    }
    
    public void setmArmPosition(Utils.Vector2D targetPose) {
        mArmPosition.set(targetPose);
    }
    
    public void setArmPositionX(double x) {
        mArmPosition.setX(x);
    }
    
    public void setArmPositionY(double y) {
        mArmPosition.setY(y);
    }
}
