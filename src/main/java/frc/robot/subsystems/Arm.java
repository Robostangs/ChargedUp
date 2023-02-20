package frc.robot.subsystems;

import java.lang.Math;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.LoggyThings.LoggyWPI_TalonFX;
import frc.robot.Constants;
import frc.robot.Utils;

public class Arm extends SubsystemBase{
    private static Arm mInstance;
    private LoggyWPI_TalonFX mShoulderMotor;
    private LoggyWPI_TalonFX mElbowMotor;

    private DoubleSupplier mElbowStickPower;
    private DoubleSupplier mShoulderStickPower;
    private BooleanSupplier mStickEnable;

    private Solenoid mShoulderBrakeSolenoid;
    private Solenoid mElbowBrakeSolenoid;
    private Solenoid mExtraSolenoid;

    private CANCoder mElbowCanCoder;
    private CANCoder mShoulderCanCoder;

    private Compressor mCompressor;

    private Utils.Vector2D mCurrentSetpoint;

    private int mShoulderLockoutCounter;
    private int mElbowLockoutCounter;

    public enum ArmPosition {
        kStowPosition,
        kIntakePosition,
        kLoadingZonePosition,
        kLowPosition,
        kMediumPosition,
        kHighPosition,
    }


    public static Arm getInstance() {
        if(mInstance == null) {
            mInstance = new Arm();
        }
        return mInstance;
    }

    public Arm() {
        mShoulderMotor = new LoggyWPI_TalonFX(Constants.Arm.shoulderMotorID, "Shoulder");
        mElbowMotor = new LoggyWPI_TalonFX(Constants.Arm.elbowMotorID, "Elbow");
    
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

        mElbowMotor.setSensorPhase(true);
        mShoulderMotor.setSensorPhase(true);

        mCompressor = new Compressor(PneumaticsModuleType.CTREPCM);
        mCompressor.enableDigital();

        mShoulderCanCoder = new CANCoder(Constants.Arm.shoulderCanCoderID);
        mElbowCanCoder = new CANCoder(Constants.Arm.elbowCanCoderID);
        
        mElbowBrakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Arm.elbowBrakeSolenoid);
        mElbowBrakeSolenoid.set(false);
        mShoulderBrakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Arm.shoulderBrakeSolenoid);
        mShoulderBrakeSolenoid.set(false);
        mExtraSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Arm.extraSolenoid);
        mExtraSolenoid.set(false);

        mCurrentSetpoint = new Utils.Vector2D(2100, -2400);

        mShoulderCanCoder.setPosition(mShoulderCanCoder.getAbsolutePosition() - (Constants.Arm.shoulderAngleSensor - Constants.Arm.shoulderAngleActual));
        mElbowCanCoder.setPosition(mElbowCanCoder.getAbsolutePosition() - (Constants.Arm.elbowAngleSensor - Constants.Arm.elbowAngleActualDifference));


        Music.insertInstrument(mElbowMotor, mShoulderMotor);

        // mElbowMotor.setInverted(true);
        // mShoulderMotor.setInverted(true);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
    }

    
    // Not needed if we use normal pid for predefined positions
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

    @Override
    public void periodic() {
        double shoulderAngle = (mShoulderMotor.getSelectedSensorPosition() / 4096) * 360;
        double elbowAngle = (mElbowMotor.getSelectedSensorPosition() / 4096) * 360;

        System.out.println("Shoulder Angle: " + shoulderAngle + " Elbow Angle: " + elbowAngle);

        Utils.Vector2D jointAngles = new Utils.Vector2D(shoulderAngle, elbowAngle);

        Utils.Vector2D handPosition = calculateHandPosition(jointAngles);
        Utils.Vector2D elbowPosition = calculateElbowPosition(jointAngles);

        double shoulderMotorPercent = (((calculateShoulderTorque(elbowPosition, handPosition) * 0.0467) / 0.05512) * 0.07) / -100;
        double elbowMotorPercent = ((calculateElbowTorque(elbowPosition, handPosition) * 0.0467) / 0.05512) / -100;

        if(mShoulderStickPower != null && mElbowStickPower != null && mStickEnable != null) {
            if(mStickEnable.getAsBoolean() == false) {
                mShoulderMotor.configPeakOutputForward(1);
                mElbowMotor.configPeakOutputForward(1);
                mElbowMotor.set(ControlMode.PercentOutput, -mElbowStickPower.getAsDouble());
                mShoulderMotor.set(ControlMode.PercentOutput, mShoulderStickPower.getAsDouble());
                // System.out.println("Stick value: " + mShoulderStickPower.getAsDouble() + " FF value: " + shoulderMotorPercent);       
            } else {
                if(mCurrentSetpoint != null) {
                    double elbowError = mCurrentSetpoint.x - mElbowMotor.getSelectedSensorPosition();
                    double shoulderError = mCurrentSetpoint.y - mShoulderMotor.getSelectedSensorPosition();

                    if(Math.abs(elbowError) < Constants.Arm.noReduceThreshold) {
                        mElbowMotor.configPeakOutputForward(0.00);
                        if(mElbowMotor.getIntegralAccumulator() > 1000) {
                            mElbowMotor.setIntegralAccumulator(0);
                        }
                        if(Math.abs(elbowError) < Constants.Arm.lockThreshold) {
                            if(mElbowLockoutCounter > 100 && mCompressor.getPressureSwitchValue()) {
                                mElbowBrakeSolenoid.set(true);
                            } else {
                                mElbowLockoutCounter++;
                            }
                        } else {
                            mElbowLockoutCounter = 0;
                        }
                    } else {
                        mElbowBrakeSolenoid.set(false);
                        mElbowMotor.configPeakOutputForward(1);
                        mElbowMotor.set(ControlMode.Position, mCurrentSetpoint.x);
                    }

                    if(mElbowBrakeSolenoid.get()) {
                        mElbowMotor.set(ControlMode.PercentOutput, 0);
                    }

                    if(Math.abs(shoulderError) < Constants.Arm.noReduceThreshold) {
                        mShoulderMotor.configPeakOutputForward(0.00);
                        if(mShoulderMotor.getIntegralAccumulator() > 1000) {
                            mShoulderMotor.setIntegralAccumulator(0);
                        }
                        if(Math.abs(shoulderError) < Constants.Arm.lockThreshold) {
                            if(mShoulderLockoutCounter > 100 && mCompressor.getPressureSwitchValue()) {
                                mShoulderBrakeSolenoid.set(true);
                            } else {
                                mShoulderLockoutCounter++;
                            }
                        } else {
                            mShoulderLockoutCounter = 0;
                        }
                    } else {
                        mShoulderBrakeSolenoid.set(false);
                        mShoulderMotor.configPeakOutputForward(1);
                        mShoulderMotor.set(ControlMode.Position, mCurrentSetpoint.y);
                    }

                    if(mShoulderBrakeSolenoid.get()) {
                        mShoulderMotor.set(ControlMode.PercentOutput, 0);
                    }
                }
            }
            
        }

        SmartDashboard.putNumber("Elbow Encoder Position", mElbowMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("Shoulder Encoder Position", mShoulderMotor.getSelectedSensorPosition());

        
        // System.out.println("Angle: " + jointAngles.y + " Predicted percent output: "+ (elbowMotorPercent) + " Actual Output: " + mElbowStickPower.getAsDouble());

    }

    public void setElbowStickSupplier(DoubleSupplier s) {
        mElbowStickPower = s;
    }

    public void setShoulderStickSupplier(DoubleSupplier s) {
        mShoulderStickPower = s;
    }
    
    public void setStickEnableSupplier(BooleanSupplier b) {
        mStickEnable = b;
    }

    public void changeSetpoint(Utils.Vector2D s) {
        mCurrentSetpoint = s;
    }
} 
