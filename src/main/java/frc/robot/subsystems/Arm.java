package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
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

    private double mElbowOffset;
    private double mShoulderOffset;
    private double shoulderAngle = 0;

    private Solenoid mShoulderBrakeSolenoid;
    private Solenoid mElbowBrakeSolenoid;
    private Solenoid mExtraSolenoid;

    private CANCoder mElbowCanCoder;
    private CANCoder mShoulderCanCoder;

    private Compressor mCompressor;

    private Utils.Vector2D mCurrentSetpoint;
    private Utils.Vector2D handPos;

    private int mShoulderLockoutCounter;
    private int mElbowLockoutCounter;
    
    // public Spark mBlinken = new Spark(0);

    Debouncer mElbowDebouncer = new Debouncer(
        0.3, DebounceType.kRising);
    Debouncer mShoulderDebouncer = new Debouncer(
        0.3, DebounceType.kRising);

    public enum ArmPosition {
        kStowPosition,
        kIntakePosition,
        kLoadingZonePosition,
        kLowPosition,
        kMediumPosition,
        kHighPosition
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
        mShoulderMotor.enableVoltageCompensation(true);
        mElbowMotor.configVoltageCompSaturation(10);
        mElbowMotor.enableVoltageCompensation(true);

        mShoulderMotor.selectProfileSlot(0, 0);
        mShoulderMotor.config_kP(0, Constants.Arm.shoulderMotorP);
        mShoulderMotor.config_kI(0, Constants.Arm.shoulderMotorI);
        mShoulderMotor.config_kD(0, Constants.Arm.shoulderMotorD);
        mShoulderMotor.config_IntegralZone(0, Constants.Arm.shoulderMotorIZone);
        
        mElbowMotor.selectProfileSlot(0, 0);
        mElbowMotor.config_kP(0, Constants.Arm.elbowMotorP);
        mElbowMotor.config_kI(0, Constants.Arm.elbowMotorI);
        mElbowMotor.config_kD(0, Constants.Arm.elbowMotorD);
        mElbowMotor.config_kF(0, Constants.Arm.elbowMotorF);
        mElbowMotor.config_IntegralZone(0, Constants.Arm.elbowMotorIZone);

        mShoulderMotor.setNeutralMode(NeutralMode.Brake);
        mElbowMotor.setNeutralMode(NeutralMode.Brake);

        mElbowMotor.setSensorPhase(false);
        mShoulderMotor.setSensorPhase(false);

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

        mCurrentSetpoint = new Utils.Vector2D(1.00,0.00);

        double correctedShoulderCanCoderPostion = mShoulderCanCoder.getAbsolutePosition() - (Constants.Arm.shoulderAngleSensor - Constants.Arm.shoulderAngleActual);
        correctedShoulderCanCoderPostion=(correctedShoulderCanCoderPostion+180)%360-180;
        
        double correctedElbowCanCoderPostion = mElbowCanCoder.getAbsolutePosition() - (Constants.Arm.elbowAngleSensor - Constants.Arm.elbowAngleActualDifference);
        correctedElbowCanCoderPostion=(correctedElbowCanCoderPostion+180)%360-180;

        mShoulderCanCoder.setPosition(correctedShoulderCanCoderPostion);
        mShoulderCanCoder.configSensorDirection(true);
        mElbowCanCoder.setPosition(correctedElbowCanCoderPostion);
        mElbowCanCoder.configSensorDirection(true);

        mShoulderMotor.configMotionCruiseVelocity(((Constants.Arm.cruiseVelocity * 4096) / 360) / 10);
        mShoulderMotor.configMotionAcceleration(((Constants.Arm.accelerationFactor * 4096) / 360) / 10);
        mShoulderMotor.configMotionSCurveStrength(Constants.Arm.smoothingFactor);

        mElbowMotor.configMotionCruiseVelocity(((Constants.Arm.cruiseVelocity * 4096) / 360) / 10);
        mElbowMotor.configMotionAcceleration(((Constants.Arm.accelerationFactor * 4096) / 360) / 10);
        mElbowMotor.configMotionSCurveStrength(Constants.Arm.smoothingFactor);

        mElbowMotor.setInverted(true);
        mShoulderMotor.setInverted(true);

        // SmartDashboard.putNumber("Set Point X", 1.0);
        // SmartDashboard.putNumber("Set Point Y", 1.0);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
    }

    
    // Not needed if we use normal pid for predefined positions
    private Utils.Vector2D calculateArmAngles(Utils.Vector2D targetPos) {
        double targetDist=Math.sqrt(targetPos.x*targetPos.x+targetPos.y*targetPos.y);
        if(targetDist>Constants.Arm.forearmLength+Constants.Arm.upperarmLength-0.05){
            double targetAngle=Math.atan2(targetPos.y,targetPos.x);
            targetPos.y=Math.sin(targetAngle)*(Constants.Arm.forearmLength+Constants.Arm.upperarmLength-0.05);
            targetPos.x=Math.cos(targetAngle)*(Constants.Arm.forearmLength+Constants.Arm.upperarmLength-0.05);

        }
        double q2 = -Math.acos((Math.pow(targetPos.x,2) + Math.pow(targetPos.y,2) - Math.pow(Constants.Arm.upperarmLength, 2) - Math.pow(Constants.Arm.forearmLength, 2))/(2*Constants.Arm.forearmLength*Constants.Arm.upperarmLength));
        double q1 = Math.atan2(targetPos.y, targetPos.x) - Math.atan2(Constants.Arm.forearmLength * Math.sin(q2), Constants.Arm.forearmLength + Constants.Arm.upperarmLength*Math.cos(q2));

        return new Utils.Vector2D(Math.toDegrees(q2), Math.toDegrees(q1));
    }

    /*
    private Utils.Vector2D calculateElbowPosition(Utils.Vector2D angles) {
        Utils.Vector2D position = new Utils.Vector2D();
        position.setX(Constants.Arm.upperarmLength * Math.cos(angles.x));
        position.setY(Constants.Arm.upperarmLength * Math.sin(angles.x));
        return position;
    }
    */

    public Utils.Vector2D calculateHandPosition(Utils.Vector2D jointAngles) {
        Utils.Vector2D position = new Utils.Vector2D();
        position.setY((Constants.Arm.upperarmLength * Math.sin(Math.toRadians(jointAngles.x))) + (Constants.Arm.forearmLength * Math.sin(Math.toRadians(jointAngles.y))));
        position.setX((Constants.Arm.upperarmLength * Math.cos(Math.toRadians(jointAngles.x))) + (Constants.Arm.forearmLength * Math.cos(Math.toRadians(jointAngles.y))));
        return position;
    }

    /*
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
    */  

    @Override
    public void periodic() {
        shoulderAngle = (mShoulderMotor.getSelectedSensorPosition() / 4096) * 360;
        double elbowAngle = (mElbowMotor.getSelectedSensorPosition() / 4096) * 360;

        Utils.Vector2D elbowPeakOutputs = new Utils.Vector2D(1.0, 1.0);
        Utils.Vector2D shoulderPeakOutputs = new Utils.Vector2D(1.0, 1.0);

        Utils.Vector2D motorAngles = calculateArmAngles(mCurrentSetpoint);
        motorAngles.x += mElbowOffset;
        motorAngles.y += mShoulderOffset;
        
        shoulderPeakOutputs.x = 1.00;
        shoulderPeakOutputs.y = 1.00;

        elbowPeakOutputs.x = 1.00;
        elbowPeakOutputs.y = 1.00;

        double elbowError = motorAngles.x*4096/360 - mElbowMotor.getSelectedSensorPosition();
        double shoulderError = motorAngles.y*4096/360 - mShoulderMotor.getSelectedSensorPosition();

        if(mCurrentSetpoint != null) {
            if(Math.abs(elbowError) < Constants.Arm.noReduceThreshold) {
                elbowPeakOutputs.y = 0.1;
                if(mElbowMotor.getIntegralAccumulator() > 1000) {
                    mElbowMotor.setIntegralAccumulator(0);
                }
                
                if(Math.abs(elbowError) < Constants.Arm.lockThreshold) {
                    if(mElbowLockoutCounter > 100) {
                        mElbowMotor.set(ControlMode.PercentOutput, 0);
                    } else {
                        mElbowLockoutCounter++;
                    }
                } else {
                    mElbowLockoutCounter = 0;
                }

            } else {
                elbowPeakOutputs.y = 1;
                mElbowLockoutCounter = 0;
                mElbowMotor.set(ControlMode.MotionMagic, motorAngles.x*4096/360);
            }

            if(Math.abs(shoulderError) < Constants.Arm.noReduceThreshold) {
                shoulderPeakOutputs.y = 0.05;
                
                if(mShoulderMotor.getIntegralAccumulator() > 1000) {
                    mShoulderMotor.setIntegralAccumulator(0);
                }

                if(Math.abs(shoulderError) < Constants.Arm.lockThreshold) {
                    if(mShoulderLockoutCounter > 100) {
                        mShoulderMotor.set(ControlMode.PercentOutput, 0);
                    } else {
                        mShoulderLockoutCounter++;
                    }
                } else {
                    mShoulderLockoutCounter = 0;
                }

            } else {
                shoulderPeakOutputs.y = 1;
                mShoulderLockoutCounter = 0;
                mShoulderMotor.set(ControlMode.MotionMagic, motorAngles.y*4096/360);
            }
        }

        if(shoulderAngle > Constants.Arm.shoulderAngleForwardSoftStop) {
            shoulderPeakOutputs.x = 0.00;
            shoulderPeakOutputs.y = 1.00;
        } else if (shoulderAngle < Constants.Arm.shoulderAngleReverseSoftStop) {
            shoulderPeakOutputs.x = 1.00;
            shoulderPeakOutputs.y = 0.00;
        }

        if(elbowAngle > Constants.Arm.elbowAngleForwardSoftStop) {
            elbowPeakOutputs.x = 0.00;
            elbowPeakOutputs.y = 1.00;
        } else if (elbowAngle < Constants.Arm.elbowAngleReverseSoftStop) {
            elbowPeakOutputs.x = 1.00;
            elbowPeakOutputs.y = 0.00;
        }
            
        mShoulderMotor.configPeakOutputForward(shoulderPeakOutputs.x);
        mShoulderMotor.configPeakOutputReverse(-shoulderPeakOutputs.y);

        mElbowMotor.configPeakOutputForward(elbowPeakOutputs.x);
        mElbowMotor.configPeakOutputReverse(-elbowPeakOutputs.y);

        mShoulderBrakeSolenoid.set(true);
        mElbowBrakeSolenoid.set(false);

        if(mShoulderDebouncer.calculate(mShoulderMotor.getMotorOutputPercent() != 0 || Math.abs(elbowError) >= Constants.Arm.lockThreshold)) {
            mShoulderBrakeSolenoid.set(true);
        } else {
            mShoulderBrakeSolenoid.set(false);
        }

        

        if(mElbowDebouncer.calculate(mElbowMotor.getMotorOutputPercent() != 0 || Math.abs(elbowError) >= Constants.Arm.lockThreshold)) {
            mElbowBrakeSolenoid.set(true);
        } else {
            mElbowBrakeSolenoid.set(false);
        }

        SmartDashboard.putNumber("Elbow Angle", elbowAngle);
        SmartDashboard.putNumber("Shoulder Angle", shoulderAngle);
        SmartDashboard.putNumber("Target Elbow Angle", motorAngles.x);
        SmartDashboard.putNumber("Target Shoulder Angle", motorAngles.y);
        
        handPos = calculateHandPosition(motorAngles);
        SmartDashboard.putNumber("Hand X", handPos.x);
        SmartDashboard.putNumber("Hand Y", handPos.y);
        // System.out.println("Angle: " + jointAngles.y + " Predicted percent output: "+ (elbowMotorPercent) + " Actual Output: " + mElbowStickPower.getAsDouble());

    }

    public double getShoulderAngle() {
        return shoulderAngle;
    }

    public double getHandPositionX() {
        return handPos.x;
    }

    public void changeSetpoint(Utils.Vector2D s) {
        mCurrentSetpoint = s;
        mShoulderOffset = 0;
        mElbowOffset = 0;
    }

    public void offsetElbow(double o) {
        mElbowOffset += o;
    }

    public void offsetShoulder(double o) {
        mShoulderOffset += o;
    }

    // public void setLight(double input) {
    //     mBlinken.set(input);
    // }
} 
