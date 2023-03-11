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
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.LoggyThings.LoggyWPI_TalonFX;
import frc.robot.Constants;
import frc.robot.Utils;
import frc.robot.Utils.LockHysteresis;
import frc.robot.Utils.Vector2D;

public class Arm extends SubsystemBase {
    private static Arm mInstance;

    private LoggyWPI_TalonFX mShoulderMotor;
    private LoggyWPI_TalonFX mElbowMotor;

    private double mYOffset;
    private double mXOffset;

    private Solenoid mShoulderBrakeSolenoid;
    private Solenoid mElbowBrakeSolenoid;
    private Solenoid mExtraSolenoid;

    private CANCoder mElbowCanCoder;
    private CANCoder mShoulderCanCoder;
    // private int shoulderTimer;

    public Compressor mCompressor;

    private Utils.Vector2D mCurrentSetpoint;
    private Utils.Vector2D mLastMotorAngles=new Utils.Vector2D(0,0);
    private Utils.Vector2D handPos;

    private int mShoulderLockoutCounter;
    private int mElbowLockoutCounter;

    private Mechanism2d mMechanismActual = new Mechanism2d(3, 3);
    private MechanismRoot2d mMechanismActualRoot;
    private MechanismLigament2d mMechanismActualShoulder;
    private MechanismLigament2d mMechanismActualElbow;
    
    private Mechanism2d mMechanismTarget = new Mechanism2d(3, 3);
    private MechanismRoot2d mMechanismTargetRoot;
    private MechanismLigament2d mMechanismTargetShoulder;
    private MechanismLigament2d mMechanismTargetElbow;

    private Mechanism2d mMechanismMotor = new Mechanism2d(3, 3);
    private MechanismRoot2d mMechanismMotorRoot;
    private MechanismLigament2d mMechanismMotorShoulder;
    private MechanismLigament2d mMechanismMotorElbow;

    Debouncer mElbowDebouncer = new Debouncer(0.3, DebounceType.kRising);
    Debouncer mShoulderDebouncer = new Debouncer(0.3, DebounceType.kRising);

    LockHysteresis mElbowHysteresis = new LockHysteresis(Constants.Arm.elbowLockThreshold, Constants.Arm.elbowLockThreshold * 8);
    LockHysteresis mShoulderHysteresis = new LockHysteresis(Constants.Arm.shoulderLockThreshold, Constants.Arm.shoulderLockThreshold * 5);

    public enum ArmPosition {
        kStowPosition,
        kIntakePositionGeneral,
        kIntakePositionUp,
        kLoadingZonePosition,
        kLowPosition,
        kMediumPosition,
        kHighPosition
    }

    public static Arm getInstance() {
        if (mInstance == null) {
            mInstance = new Arm();
        }
        return mInstance;
    }

    public Arm() {
        mShoulderMotor = new LoggyWPI_TalonFX(Constants.Arm.shoulderMotorID, "/Shoulder/");
        mElbowMotor = new LoggyWPI_TalonFX(Constants.Arm.elbowMotorID, "/Elbow/");

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
        mElbowBrakeSolenoid.set(true);
        mShoulderBrakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Arm.shoulderBrakeSolenoid);
        mShoulderBrakeSolenoid.set(true);
        mExtraSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Arm.extraSolenoid);
        mExtraSolenoid.set(false);

        // mCurrentSetpoint = new Utils.Vector2D(0.4, 0.1);

        double correctedShoulderCanCoderPostion = mShoulderCanCoder.getAbsolutePosition()
                - (Constants.Arm.shoulderAngleSensor + Constants.Arm.shoulderAngleActual);
        correctedShoulderCanCoderPostion = (correctedShoulderCanCoderPostion + 180) % 360 - 180;

        double correctedElbowCanCoderPostion = mElbowCanCoder.getAbsolutePosition()
                - (Constants.Arm.elbowAngleSensor + Constants.Arm.elbowAngleActualDifference);
        correctedElbowCanCoderPostion = (correctedElbowCanCoderPostion + 180) % 360 - 180;

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

        mShoulderMotor.setNeutralMode(NeutralMode.Coast);
        mElbowMotor.setNeutralMode(NeutralMode.Coast);

        mElbowMotor.setInverted(true);
        mShoulderMotor.setInverted(true);
        mShoulderMotor.setSelectedSensorPosition((mShoulderCanCoder.getPosition()*109.333 * 4096) / 360);
        mElbowMotor.setSelectedSensorPosition((mElbowCanCoder.getPosition() *61.134* 4096) / 360);
        mMechanismActualRoot = mMechanismActual.getRoot("ArmRoot", 1.5, 0);
        mMechanismActualShoulder = mMechanismActualRoot.append(new MechanismLigament2d("Shoulder", Constants.Arm.upperarmLength, 0, 2, new Color8Bit(Color.kPurple)));
        mMechanismActualElbow = mMechanismActualShoulder.append(new MechanismLigament2d("Elbow", Constants.Arm.forearmLength, 0, 2, new Color8Bit(Color.kPurple)));
    
        mMechanismTargetRoot = mMechanismTarget.getRoot("ArmRoot", 1.5, 0);
        mMechanismTargetShoulder = mMechanismTargetRoot.append(new MechanismLigament2d("Shoulder", Constants.Arm.upperarmLength, 0, 2, new Color8Bit(Color.kPurple)));
        mMechanismTargetElbow = mMechanismTargetShoulder.append(new MechanismLigament2d("Elbow", Constants.Arm.forearmLength, 0, 2, new Color8Bit(Color.kPurple)));

        mMechanismMotorRoot = mMechanismMotor.getRoot("ArmRoot", 1.5, 0);
        mMechanismMotorShoulder = mMechanismMotorRoot.append(new MechanismLigament2d("Shoulder", Constants.Arm.upperarmLength, 0, 2, new Color8Bit(Color.kPurple)));
        mMechanismMotorElbow = mMechanismMotorShoulder.append(new MechanismLigament2d("Elbow", Constants.Arm.forearmLength, 0, 2, new Color8Bit(Color.kPurple)));

    }

    // Not needed if we use normal pid for predefined positions
    private Utils.Vector2D calculateArmAngles(Utils.Vector2D targetPos) {
        double targetDist = Math.sqrt(targetPos.x * targetPos.x + targetPos.y * targetPos.y);
        if (targetDist > Constants.Arm.forearmLength + Constants.Arm.upperarmLength - 0.05) {
            double targetAngle = Math.atan2(targetPos.y, targetPos.x);
            targetPos.y = Math.sin(targetAngle) * (Constants.Arm.forearmLength + Constants.Arm.upperarmLength - 0.05);
            targetPos.x = Math.cos(targetAngle) * (Constants.Arm.forearmLength + Constants.Arm.upperarmLength - 0.05);
        }
        if(Math.abs(targetPos.x) > Constants.Hand.maxFrameExtension.x) {
            targetPos.x = Math.signum(targetPos.x)*Constants.Hand.maxFrameExtension.x;
        } 

        if(targetPos.y > Constants.Hand.maxFrameExtension.y) {
            targetPos.y = Constants.Hand.maxFrameExtension.y;
        }
        if(targetPos.y < -0.15) {
            targetPos.y = -0.14;
        }

        double q2 = -Math.acos((Math.pow(targetPos.x, 2) + Math.pow(targetPos.y, 2)
                - Math.pow(Constants.Arm.upperarmLength, 2) - Math.pow(Constants.Arm.forearmLength, 2))
                / (2 * Constants.Arm.forearmLength * Constants.Arm.upperarmLength));
        double q1 = Math.atan2(targetPos.y, targetPos.x) - Math.atan2(Constants.Arm.forearmLength * Math.sin(q2),
                Constants.Arm.forearmLength + Constants.Arm.upperarmLength * Math.cos(q2));

        return new Utils.Vector2D(Math.toDegrees(q2), Math.toDegrees(q1));
    }

    public Utils.Vector2D calculateHandPosition(Utils.Vector2D jointAngles) {
        Utils.Vector2D position = new Utils.Vector2D();
        position.setY((Constants.Arm.upperarmLength * Math.sin(Math.toRadians(jointAngles.y)))
                + (Constants.Arm.forearmLength * Math.sin(Math.toRadians(jointAngles.x + jointAngles.y))));
        position.setX((Constants.Arm.upperarmLength * Math.cos(Math.toRadians(jointAngles.y)))
                + (Constants.Arm.forearmLength * Math.cos(Math.toRadians(jointAngles.x + jointAngles.y))));
        return position;
    }

    @Override
    public void periodic() {
        double shoulderAngle = (mShoulderMotor.getSelectedSensorPosition() / 4096) * 360/109.33;
        double elbowAngle = ((mElbowMotor.getSelectedSensorPosition() / 4096) * 360/61.134);

        Utils.Vector2D elbowPeakOutputs = new Utils.Vector2D(1.0, 1.0);
        Utils.Vector2D shoulderPeakOutputs = new Utils.Vector2D(1.0, 1.0);

        /*
        Utils.Vector2D motorAngles = calculateArmAngles(mCurrentSetpoint);

        //motorAngles.x += mYOffset;
        //motorAngles.y += mXOffset;

        shoulderPeakOutputs.x = 1.00;
        shoulderPeakOutputs.y = 1.00;

        elbowPeakOutputs.x = 1.00;
        elbowPeakOutputs.y = 1.00;

        double elbowError = motorAngles.x * 4096 / 360 - mElbowMotor.getSelectedSensorPosition();
        double shoulderError = motorAngles.y * 4096 / 360 - mShoulderMotor.getSelectedSensorPosition();

        if (mCurrentSetpoint != null) {
            if(motorAngles.x != mLastMotorAngles.x){
                mElbowHysteresis.reset();
            }
            if(motorAngles.y != mLastMotorAngles.y){
                mShoulderHysteresis.reset();
            }
            if (Math.abs(elbowError) < Constants.Arm.noReduceThreshold) {
                elbowPeakOutputs.y = 0.12   ;
            }
            if (Math.abs(elbowError) < Constants.Arm.noReduceThreshold
                    && mElbowDebouncer.calculate(mElbowHysteresis.calculate(Math.abs(elbowError)))) {
                mElbowMotor.set(ControlMode.PercentOutput, 0);
                mElbowBrakeSolenoid.set(false);
            } else {
                mElbowBrakeSolenoid.set(true);
                mElbowMotor.set(ControlMode.MotionMagic, motorAngles.x * 4096 / 360  );
            }

            if (Math.abs(shoulderError) < Constants.Arm.noReduceThreshold) {
                shoulderPeakOutputs.y = 0.1;
            }
            if ((Math.abs(shoulderError) < Constants.Arm.noReduceThreshold
                    && mShoulderDebouncer.calculate(mShoulderHysteresis.calculate(Math.abs(shoulderError)))
                    )) {
                mShoulderMotor.set(ControlMode.PercentOutput, 0);
                mShoulderBrakeSolenoid.set(false);
            } else {
                mShoulderBrakeSolenoid.set(true);
                mShoulderMotor.set(ControlMode.MotionMagic, motorAngles.y * 4096 / 360);
            }
            */

            if (shoulderAngle > Constants.Arm.shoulderAngleForwardSoftStop) {
                shoulderPeakOutputs.x = 0.00;
                shoulderPeakOutputs.y = 1.00;
            } else if (shoulderAngle < Constants.Arm.shoulderAngleReverseSoftStop) {
                shoulderPeakOutputs.x = 1.00;
                shoulderPeakOutputs.y = 0.00;
            }

            if (elbowAngle > Constants.Arm.elbowAngleForwardSoftStop) {
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

            // SmartDashboard.putNumber("Elbow Error", elbowError);
            // SmartDashboard.putNumber("Shoulder Error", shoulderError);

            // SmartDashboard.putNumber("Elbow Angle", elbowAngle);
            // SmartDashboard.putNumber("Shoulder Angle", shoulderAngle);
            // SmartDashboard.putNumber("Target Elbow Angle", motorAngles.x);
            // SmartDashboard.putNumber("Target Shoulder Angle", motorAngles.y);

            handPos = calculateHandPosition(new Utils.Vector2D(elbowAngle,shoulderAngle));
            // SmartDashboard.putNumber("Hand X", handPos.x);
            // SmartDashboard.putNumber("Hand Y", handPos.y);

            // SmartDashboard.putNumber("SetPoint Hand X", mCurrentSetpoint.x);
            // SmartDashboard.putNumber("SetPoint Hand Y", mCurrentSetpoint.y);
            // mLastMotorAngles = motorAngles.clone();

            // SmartDashboard.putBoolean("ElbowLock", getElbowLocked());
            // SmartDashboard.putBoolean("ShoulderLock", getShoulderLocked());
            // SmartDashboard.putData(Arm.getInstance());

            SmartDashboard.putNumber("Shoulder Cancoder", mShoulderCanCoder.getPosition());
            SmartDashboard.putNumber("Shoulder Motor Encoder", ((mShoulderMotor.getSelectedSensorPosition() / 4096) * 360/109.33));
            SmartDashboard.putNumber("Elbow Cancoder", mElbowCanCoder.getPosition());
            SmartDashboard.putNumber("Elbow Motor Encoder", -((mElbowMotor.getSelectedSensorPosition() / 4096) * 360/61.134));
        
            mMechanismActualElbow.setAngle(elbowAngle);
            mMechanismActualShoulder.setAngle(shoulderAngle);

            mMechanismTargetElbow.setAngle(mCurrentSetpoint.x);
            mMechanismTargetShoulder.setAngle(mCurrentSetpoint.y);

            mMechanismMotorElbow.setAngle(mElbowMotor.getSelectedSensorPosition());
            mMechanismMotorShoulder.setAngle(mShoulderMotor.getSelectedSensorPosition());

            SmartDashboard.putData("ArmMechanism/Actual", mMechanismActual);
            SmartDashboard.putData("ArmMechanism/Target", mMechanismTarget);
            SmartDashboard.putData("ArmMechanism/Motor", mMechanismMotor);
        }
    }


    public double getHandPositionX() {
        return handPos.x;
    }

    public void changeSetpoint(Utils.Vector2D currentAngles, Utils.Vector2D setpoint) {
        Vector2D mCurrentAngle = currentAngles;
        Vector2D mSetpoint = setpoint;

        if(mSetpoint != null && mCurrentAngle != null) {

            if(Math.abs(mSetpoint.x-mCurrentAngle.x) > 0.2) {
                mElbowBrakeSolenoid.set(true);
                mElbowMotor.set(ControlMode.MotionMagic, mSetpoint.x);
            }

            if(Math.abs(mSetpoint.y-mCurrentAngle.y) > 0.2) {
                mShoulderBrakeSolenoid.set(true);
                mShoulderMotor.set(ControlMode.MotionMagic, mSetpoint.y);
            }

            if(mElbowHysteresis.calculate(mElbowMotor.getClosedLoopError())) {
                mElbowBrakeSolenoid.set(false);
            }
            
            if(mShoulderHysteresis.calculate(mShoulderMotor.getClosedLoopError())) {
                mShoulderBrakeSolenoid.set(false);
            }
        }

        
        // shoulderTimer = 0;
    }

    public void offsetY(double yOffset) {
        mCurrentSetpoint.add(0,yOffset);

    }

    public void offsetX(double xOffset) {
        mCurrentSetpoint.add(xOffset,0);
    }

    public double getAbsolutePositionElbow() {
        return mElbowCanCoder.getPosition();
    }

    public double getAbsolutePositionShoulder() {
        return mShoulderCanCoder.getPosition();
    }

    public void setSound(double hertz) {
        mShoulderMotor.set(ControlMode.MusicTone, hertz);
        mElbowMotor.set(ControlMode.MusicTone, hertz);
    }
    public boolean getElbowLocked() {
        return !mElbowBrakeSolenoid.get();
    }

    public boolean getShoulderLocked() {
        return !mShoulderBrakeSolenoid.get();
    }

    // public void setLight(double input) {
    // mBlinken.set(input);
    // }
}
