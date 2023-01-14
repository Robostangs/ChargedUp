package frc.robot.subsystems;

import java.lang.Math;
import java.util.ArrayList;

import com.revrobotics.CANSparkMax.IdleMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.LoggyThings.LoggyCANSparkMax;
import frc.LoggyThings.LoggyWPI_TalonFX;
import frc.robot.Constants;
import frc.robot.Utils;

public class Arm extends SubsystemBase{
    private static Arm mInstance;
    private LoggyWPI_TalonFX mShoulderMotor;
    private LoggyWPI_TalonFX mElbowMotor;

    private Utils.Vector2D mArmPosition;
    private Utils.Vector2D mElbowPosition;

    public static Arm getInstance() {
        if(mInstance == null) {
            mInstance = new Arm();
        }
        return mInstance;
    }

    public Arm() {
        mShoulderMotor = new LoggyWPI_TalonFX(Constants.Arm.shoulderMotorID, "Shoulder");
        mElbowMotor = new LoggyWPI_TalonFX(Constants.Arm.shoulderMotorID, "Elbow");
    
        mShoulderMotor.configVoltageCompSaturation(10);
        mElbowMotor.configVoltageCompSaturation(10);

        mShoulderMotor.config_kP(0, Constants.Arm.shoulderMotorP);
        mShoulderMotor.config_kI(0, Constants.Arm.shoulderMotorI);
        mShoulderMotor.config_kD(0, Constants.Arm.shoulderMotorD);
        mShoulderMotor.config_IntegralZone(0, Constants.Arm.shoulderMotorIZone);

        mElbowMotor.config_kP(0, Constants.Arm.elbowMotorP);
        mElbowMotor.config_kI(0, Constants.Arm.elbowMotorI);
        mElbowMotor.config_kD(0, Constants.Arm.elbowMotorD);
        mElbowMotor.config_IntegralZone(0, Constants.Arm.elbowMotorIZone);

        mShoulderMotor.setNeutralMode(NeutralMode.Brake);
        mElbowMotor.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Arm Position X", this.mArmPosition::getX, this.mArmPosition::setX);
        builder.addDoubleProperty("Arm Position Y", this.mArmPosition::getY, this.mArmPosition::setY);
        builder.addDoubleProperty( "Shoulder Rotation", this::getShoulderRotation, null);
        builder.addDoubleProperty( "Elbow Rotation", this::getElbowRotation, null);
    }


    private Utils.Vector2D calculateArmPosition(Utils.Vector2D targetPos) {
        double q2 = Math.acos(Math.pow(targetPos.x,2) + Math.pow(targetPos.y,2) - Math.pow(Constants.Arm.upperarmLength, 2) - Math.pow(Constants.Arm.forearmLength, 2));
        double q1 = Math.atan2(targetPos.y, targetPos.x) - Math.atan2(Constants.Arm.forearmLength * Math.sin(q2), Constants.Arm.forearmLength + Constants.Arm.upperarmLength*Math.cos(q2));
        return new Utils.Vector2D(q1, q1);
    }

    private void calculateElbowPosition(Utils.Vector2D angles) {
        mElbowPosition.setX(Constants.Arm.upperarmLength * Math.cos(angles.x));
        mElbowPosition.setY(Constants.Arm.upperarmLength * Math.cos(angles.y));
    }

    private double calculateShoulderTorque() {
        double mass = Constants.Arm.elbowMass + Constants.Hand.handMass + Constants.Arm.forearmMass + Constants.Arm.upperarmMass;

        Utils.Vector2D forearmCenterOfMass = new Utils.Vector2D(mElbowPosition.getX() / 2, mElbowPosition.getY() / 2);
        Utils.Vector2D upperarmCenterOfMass = new Utils.Vector2D((mElbowPosition.getX() + mArmPosition.getX()) / 2, (mElbowPosition.getY() + mArmPosition.getAngle()) / 2 );
        double armCenterOfMassX = (mElbowPosition.getX() * Constants.Arm.elbowMass + mArmPosition.getX() * Constants.Hand.handMass +
                                   forearmCenterOfMass.getX() * Constants.Arm.forearmMass + upperarmCenterOfMass.getX() * Constants.Arm.upperarmMass) /
                                  mass;
        
        double armCenterOfMassY = (mElbowPosition.getY() * Constants.Arm.elbowMass + mArmPosition.getY() * Constants.Hand.handMass +
                                   forearmCenterOfMass.getY() * Constants.Arm.forearmMass + upperarmCenterOfMass.getY() * Constants.Arm.upperarmMass) /
                                  mass;
        
        double distance = Math.sqrt(Math.pow(armCenterOfMassX, 2) + Math.pow(armCenterOfMassY, 2));
        double angle = Math.atan(armCenterOfMassY / armCenterOfMassX);
        double torqueOfGravity = (mass * 9.81) * distance * Math.cos(angle);

        return torqueOfGravity;
    }

    private double calculateElbowTorque() {
        double mass = Constants.Arm.elbowMass + Constants.Hand.handMass;

        Utils.Vector2D forearmCenterOfMass = new Utils.Vector2D(mElbowPosition.getX() / 2, mElbowPosition.getY() / 2);
        double armCenterOfMassX = (mArmPosition.getX() * Constants.Hand.handMass + forearmCenterOfMass.getX() * Constants.Arm.forearmMass) /
                                  mass;
        
        double armCenterOfMassY = (mArmPosition.getY() * Constants.Hand.handMass + forearmCenterOfMass.getY() * Constants.Arm.forearmMass) /
                                  mass;
        
        double distance = Math.sqrt(Math.pow(armCenterOfMassX, 2) + Math.pow(armCenterOfMassY, 2));
        double angle = Math.atan(armCenterOfMassY / armCenterOfMassX);
        double torqueOfGravity = (mass * 9.81) * distance * Math.cos(angle);

        return torqueOfGravity;
    }

    @Override
    public void periodic() {
        Utils.Vector2D jointAngles = calculateArmPosition(mArmPosition);
        calculateElbowPosition(jointAngles);

        double shoulderMotorVoltage = (calculateShoulderTorque() * 0.0467) / 0.05512;
        double elbowMotorVoltage = (calculateElbowTorque() * 0.0467) / 0.05512;

        mShoulderMotor.set(ControlMode.Position, jointAngles.getX() / Math.PI, DemandType.ArbitraryFeedForward, shoulderMotorVoltage / 10);
        mElbowMotor.set(ControlMode.Position, jointAngles.getY() / Math.PI, DemandType.ArbitraryFeedForward, elbowMotorVoltage / 10);
    }

    public Utils.Vector2D getmArmPosition() {
        return mArmPosition;
    }
    
    
    public double getShoulderRotation() {
        return mShoulderMotor.getSelectedSensorPosition();
    }
    
    public double getElbowRotation() {
        return mElbowMotor.getSelectedSensorPosition();
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
