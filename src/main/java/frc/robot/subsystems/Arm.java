package frc.robot.subsystems;

import java.lang.Math;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.LoggyThings.LoggyWPI_TalonFX;
import frc.robot.Constants;
import frc.robot.Utils;
import frc.robot.Utils.Vector2D;

public class Arm extends SubsystemBase{
    private static Arm mInstance;
    private LoggyWPI_TalonFX mShoulderMotor;
    private LoggyWPI_TalonFX mElbowMotor;

    private CANCoder mArmCanCoder, mElbowCanCoder;
    private Utils.Vector2D mEncoderValues = new Vector2D(0,0);
    private Solenoid mElbowBrakeSolenoid, mArmBrakeSolenoid;
    private DoubleSupplier mStickPower;
    private BooleanSupplier mStickEnable;


    public static Arm getInstance() {
        if(mInstance == null) {
            mInstance = new Arm();
        }
        return mInstance;
    }

    public enum ArmState {
        khighPosition,
        kmediumPosition,
        klowPosition,
        kStowedPosition,
        kIntakePosition,
        kLoadingZonePosition
    }

    public Arm() {
        mShoulderMotor = new LoggyWPI_TalonFX(Constants.Arm.shoulderMotorID, "Shoulder");
        mElbowMotor = new LoggyWPI_TalonFX(Constants.Arm.elbowMotorID, "Elbow");
    
        mShoulderMotor.configVoltageCompSaturation(10);
        mElbowMotor.configVoltageCompSaturation(10);

        mShoulderMotor.config_kP(0, Constants.Arm.shoulderMotorP);
        mShoulderMotor.config_kI(0, Constants.Arm.shoulderMotorI);
        mShoulderMotor.config_kD(0, Constants.Arm.shoulderMotorD);
        mShoulderMotor.config_kF(0, Constants.Arm.shoulderMotorF);
        mShoulderMotor.config_IntegralZone(0, Constants.Arm.shoulderMotorIZone);

        mElbowMotor.config_kP(0, Constants.Arm.elbowMotorP);
        mElbowMotor.config_kI(0, Constants.Arm.elbowMotorI);
        mElbowMotor.config_kD(0, Constants.Arm.elbowMotorD);
        mElbowMotor.config_IntegralZone(0, Constants.Arm.elbowMotorIZone);

        mShoulderMotor.setNeutralMode(NeutralMode.Brake);
        mElbowMotor.setNeutralMode(NeutralMode.Brake);
        
        mArmCanCoder = new CANCoder(Constants.Arm.mArmCanCoderID);
        mElbowCanCoder = new CANCoder(Constants.Arm.mElbowCanCoderID);

        mArmBrakeSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.Arm.mArmBrakeID);
        mArmBrakeSolenoid.set(false);
        mElbowBrakeSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.Arm.mElbowBrakeSolenoid);
        mElbowBrakeSolenoid.set(false);

        mElbowMotor.setInverted(true);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty( "Shoulder Rotation", this::getIncludedShoulderRotation, null);
        builder.addDoubleProperty( "Elbow Rotation", this::getIncludedElbowRotation, null);
    }


    private Utils.Vector2D calculateArmAngles(Utils.Vector2D targetPos) {
        double q2 = Math.acos(Math.pow(targetPos.x,2) + Math.pow(targetPos.y,2) - Math.pow(Constants.Arm.upperarmLength, 2) - Math.pow(Constants.Arm.forearmLength, 2));
        double q1 = Math.atan2(targetPos.y, targetPos.x) - Math.atan2(Constants.Arm.forearmLength * Math.sin(q2), Constants.Arm.forearmLength + Constants.Arm.upperarmLength*Math.cos(q2));
        return new Utils.Vector2D(q1, q1);
    }

    private Utils.Vector2D calculateElbowPosition(Utils.Vector2D angles) {
        Utils.Vector2D position = new Utils.Vector2D();
        position.setX(Constants.Arm.upperarmLength * Math.cos(angles.x));
        position.setY(Constants.Arm.upperarmLength * Math.sin(angles.x));
        return position;
    }

    public Utils.Vector2D calculateHandPosition(Utils.Vector2D jointAngles) {
        Utils.Vector2D position = new Utils.Vector2D();
        position.setX((Constants.Arm.upperarmLength * Math.sin(jointAngles.x)) + (Constants.Arm.forearmLength * Math.sin(jointAngles.y)));
        position.setY((Constants.Arm.upperarmLength * Math.cos(jointAngles.x)) + (Constants.Arm.forearmLength * Math.cos(jointAngles.y)));
        return position;
    }

    private double calculateShoulderTorque(Utils.Vector2D elbowPosition, Utils.Vector2D handPosition) {
        double mass = Constants.Arm.elbowMass + Constants.Hand.handMass + Constants.Arm.forearmMass + Constants.Arm.upperarmMass;

        Utils.Vector2D forearmCenterOfMass = new Utils.Vector2D(elbowPosition.getX() / 2, elbowPosition.getY() / 2);
        Utils.Vector2D upperarmCenterOfMass = new Utils.Vector2D((elbowPosition.getX() + handPosition.getX()) / 2, (elbowPosition.getY() + handPosition.getAngle()) / 2 );
        double armCenterOfMassX = (elbowPosition.getX() * Constants.Arm.elbowMass + handPosition.getX() * Constants.Hand.handMass +
                                   forearmCenterOfMass.getX() * Constants.Arm.forearmMass + upperarmCenterOfMass.getX() * Constants.Arm.upperarmMass) /
                                  mass;
        
        double armCenterOfMassY = (elbowPosition.getY() * Constants.Arm.elbowMass + handPosition.getY() * Constants.Hand.handMass +
                                   forearmCenterOfMass.getY() * Constants.Arm.forearmMass + upperarmCenterOfMass.getY() * Constants.Arm.upperarmMass) /
                                  mass;
        
        double distance = Math.sqrt(Math.pow(armCenterOfMassX, 2) + Math.pow(armCenterOfMassY, 2));
        double angle = Math.atan(armCenterOfMassY / armCenterOfMassX);
        double torqueOfGravity = (mass * 9.81) * distance * Math.cos(angle);

        return torqueOfGravity;
    }

    private double calculateElbowTorque(Utils.Vector2D elbowPosition, Utils.Vector2D handPosition) {
        double mass = Constants.Arm.elbowMass + Constants.Hand.handMass;
        
        Utils.Vector2D forearmCenterOfMass = new Utils.Vector2D(elbowPosition.getX() / 2, elbowPosition.getY() / 2);
        double armCenterOfMassX = (handPosition.getX() * Constants.Hand.handMass + forearmCenterOfMass.getX() * Constants.Arm.forearmMass) /
                                  mass;
        
        double armCenterOfMassY = (handPosition.getY() * Constants.Hand.handMass + forearmCenterOfMass.getY() * Constants.Arm.forearmMass) /
                                  mass;

        double distance = Math.sqrt(Math.pow(armCenterOfMassX, 2) + Math.pow(armCenterOfMassY, 2));
        double angle = Math.atan(armCenterOfMassY / armCenterOfMassX);
        double torqueOfGravity = (mass * 9.81) * distance * Math.cos(angle);

        return torqueOfGravity;
    }

    private Utils.Vector2D getEncoderValue() {
        if(mArmCanCoder.getStickyFaults(null) != null) {
            mEncoderValues.setX(getOutsideShoulderRotation());
        } else {
            double value = getIncludedShoulderRotation() + mEncoderValues.getX();
            if(value >= 360) {
                mEncoderValues.setX(value-360);
            } else {
                mEncoderValues.setX(value);
            }
        }

        if(mElbowCanCoder.getStickyFaults(null) != null) {
            mEncoderValues.setY(getOutsideElbowRotation());
        } else {
            double value = getIncludedElbowRotation() + mEncoderValues.getY();
            if(value >= 360) {
                mEncoderValues.setY(value-360);
            } else {
                mEncoderValues.setY(value);
            }
        }
        return mEncoderValues;
    }

    

    @Override
    public void periodic() {
        double totalArmLengths = Constants.Arm.forearmLength + Constants.Arm.upperarmLength;
        double shoulderAngle = (mShoulderMotor.getSelectedSensorPosition() / 4096) * 360;
        double elbowAngle = (mElbowMotor.getSelectedSensorPosition() / 4096) * 360;

        Utils.Vector2D jointAngles = new Utils.Vector2D(shoulderAngle, elbowAngle);

        Utils.Vector2D handPosition = calculateHandPosition(jointAngles);
        Utils.Vector2D elbowPosition = calculateElbowPosition(jointAngles);

        double shoulderMotorVoltage = ((calculateShoulderTorque(elbowPosition, handPosition) * 0.0467) / 0.05512) * 0.7;
        double elbowMotorVoltage = (calculateElbowTorque(elbowPosition, handPosition) * 0.0467) / 0.05512;
        double elbowMotorVoltagePercent = elbowMotorVoltage / 100;

        if(mStickPower != null && mStickEnable != null) {
            if(mStickEnable.getAsBoolean() == true) {
                mElbowMotor.set(ControlMode.PercentOutput, mStickPower.getAsDouble());
            } else {
                mElbowMotor.set(ControlMode.PercentOutput, elbowMotorVoltagePercent / Constants.Arm.elbowMotorCompensationFactor);
            }
            
        }
        
        System.out.println("Angle: " + jointAngles.y + " Predicted percent output: "+ (elbowMotorVoltage / 100) + " Actual Output: " + mStickPower.getAsDouble());

    }

    public void setStickSupplier(DoubleSupplier s) {
        mStickPower = s;
    }
    
    public void setStickEnableSupplier(BooleanSupplier b) {
        mStickEnable = b;
    }

    public double getIncludedShoulderRotation() {
        return mShoulderMotor.getSelectedSensorPosition();
    }
    
    public double getIncludedElbowRotation() {
        return mElbowMotor.getSelectedSensorPosition();
    }
    
    public double getOutsideShoulderRotation() {
        return mArmCanCoder.getPosition();
    }
    
    public double getOutsideElbowRotation() {
        return mElbowCanCoder.getPosition();
    }

    public void setBrakeMode(boolean brake) {
        
    }

    public void setShoulderMotorPower(double power) {
        mShoulderMotor.set(ControlMode.PercentOutput, power);
    }

    public void setElbowMotorPower(double power) {
        mElbowMotor.set(ControlMode.PercentOutput, power);
    }

    public boolean isSwitchShoulder() {
        return mShoulderMotor.isFwdLimitSwitchClosed() == 1 ? true : false;
    }

    public boolean isSwitchElbow() {
        return mElbowMotor.isRevLimitSwitchClosed() == 1 ? true : false;
    }

    public TalonFX[] getTalonFXs() {
        final LoggyWPI_TalonFX[] talons = { mShoulderMotor, mElbowMotor };
        return talons;
    }
} 
