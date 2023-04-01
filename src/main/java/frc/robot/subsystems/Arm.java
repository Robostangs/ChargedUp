package frc.robot.subsystems;

import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.ArmTrajectoryPlanner.ArmTrajectoryPlanner;
import frc.LoggyThings.LoggyWPI_TalonFX;
import frc.robot.Constants;
import frc.robot.Utils;
import frc.robot.Utils.LockHysteresis;
import frc.robot.Utils.Vector2D;

public class Arm extends SubsystemBase {
    private static Arm mInstance;

    private LoggyWPI_TalonFX mShoulderMotor;
    private LoggyWPI_TalonFX mElbowMotor;

    private Solenoid mShoulderBrakeSolenoid;
    private Solenoid mElbowBrakeSolenoid;
    private Solenoid mExtraSolenoid;

    private CANCoder mElbowCanCoder;
    private CANCoder mShoulderCanCoder;
    // private int shoulderTimer;

    public Compressor mCompressor;


    private Mechanism2d mMechanismActual = new Mechanism2d(Constants.Hand.maxFrameExtension.x*2, Constants.Hand.maxFrameExtension.y);
    private MechanismRoot2d mMechanismActualRoot;
    private MechanismLigament2d mMechanismActualShoulder;
    private MechanismLigament2d mMechanismActualElbow;

    private Mechanism2d mMechanismTarget = new Mechanism2d(Constants.Hand.maxFrameExtension.x*2, Constants.Hand.maxFrameExtension.y);
    private MechanismRoot2d mMechanismTargetRoot;
    private MechanismLigament2d mMechanismTargetShoulder;
    private MechanismLigament2d mMechanismTargetElbow;

    private Mechanism2d mMechanismProfile = new Mechanism2d(Constants.Hand.maxFrameExtension.x*2, Constants.Hand.maxFrameExtension.y);
    private MechanismRoot2d mMechanismProfileRoot;
    private MechanismLigament2d mMechanismProfileShoulder;
    private MechanismLigament2d mMechanismProfileElbow;

    private Mechanism2d mMechanismMotor = new Mechanism2d(Constants.Hand.maxFrameExtension.x*2, Constants.Hand.maxFrameExtension.y);
    private MechanismRoot2d mMechanismMotorRoot;
    private MechanismLigament2d mMechanismMotorShoulder;
    private MechanismLigament2d mMechanismMotorElbow;

    private Vector2D handPosFromMotors;
    private Vector2D handPos = new Vector2D();

    public Vector2D getHandPos() {
        return handPos;
    }
    Debouncer mElbowDebouncer = new Debouncer(0.3, DebounceType.kRising);
    Debouncer mShoulderDebouncer = new Debouncer(0.3, DebounceType.kRising);

    LockHysteresis mElbowHysteresis = new LockHysteresis(Constants.Arm.elbowLockThreshold,
            Constants.Arm.elbowLockThreshold * 8);
    LockHysteresis mShoulderHysteresis = new LockHysteresis(Constants.Arm.shoulderLockThreshold,
            Constants.Arm.shoulderLockThreshold * 5);

    public enum ArmPosition {
        kStartPosition,
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

    private Arm() {
        mShoulderMotor = new LoggyWPI_TalonFX(Constants.Arm.shoulderMotorID, "/Shoulder/Motor/");
        mElbowMotor = new LoggyWPI_TalonFX(Constants.Arm.elbowMotorID, "/Elbow/Motor/");

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
        setElbowLock(false);
        mShoulderBrakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Arm.shoulderBrakeSolenoid);
        setShoulderLock(false);
        mExtraSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Arm.extraSolenoid);
        mExtraSolenoid.set(false);

        mShoulderCanCoder.configSensorDirection(false);
        mElbowCanCoder.configSensorDirection(false);
        DataLogManager.log("SHOULDER ABSOLUTE POSITION "+mShoulderCanCoder.getAbsolutePosition());
        double correctedShoulderCanCoderPostion = mShoulderCanCoder.getAbsolutePosition()
                - Constants.Arm.shoulderAngleSensor + Constants.Arm.shoulderAngleActual;
        correctedShoulderCanCoderPostion = Utils.clampDegreeMeasurement(correctedShoulderCanCoderPostion);
        DataLogManager.log("CORRECTED SHOULDER POSITION "+correctedShoulderCanCoderPostion);

        DataLogManager.log("ELBOW ABSOLUTE POSITION "+mElbowCanCoder.getAbsolutePosition());

        double correctedElbowCanCoderPostion = mElbowCanCoder.getAbsolutePosition()
                - Constants.Arm.elbowAngleSensor + Constants.Arm.elbowAngleActualDifference;
        correctedElbowCanCoderPostion = Utils.clampDegreeMeasurement(correctedElbowCanCoderPostion);
        DataLogManager.log("CORRECTED ELBOW POSITION "+correctedElbowCanCoderPostion);

        mShoulderCanCoder.setPosition(correctedShoulderCanCoderPostion);
        mElbowCanCoder.setPosition(correctedElbowCanCoderPostion);

        mShoulderMotor.configMotionCruiseVelocity(Constants.Arm.shoulderCruiseVelocity);
        mShoulderMotor.configMotionAcceleration(Constants.Arm.shoulderAccelerationFactor);
        mShoulderMotor.configMotionSCurveStrength(Constants.Arm.smoothingFactor);

        mElbowMotor.configMotionCruiseVelocity(Constants.Arm.elbowCruiseVelocity);
        mElbowMotor.configMotionAcceleration(Constants.Arm.elbowAccelerationFactor);
        mElbowMotor.configMotionSCurveStrength(Constants.Arm.smoothingFactor);

        mShoulderMotor.configMotionProfileTrajectoryInterpolationEnable(true);
        mShoulderMotor.configMotionProfileTrajectoryPeriod((int)(ArmTrajectoryPlanner.sampleTime*1000));
        mShoulderMotor.changeMotionControlFramePeriod((int)(ArmTrajectoryPlanner.sampleTime*1000/2));
        mElbowMotor.configMotionProfileTrajectoryInterpolationEnable(true);
        mElbowMotor.configMotionProfileTrajectoryPeriod((int)(ArmTrajectoryPlanner.sampleTime*1000));
        mElbowMotor.changeMotionControlFramePeriod((int)(ArmTrajectoryPlanner.sampleTime*1000/2));


        mElbowMotor.setStatusFramePeriod(StatusFrame.Status_10_Targets, 20);
        mShoulderMotor.setStatusFramePeriod(StatusFrame.Status_10_Targets, 20);

        mShoulderMotor.setNeutralMode(NeutralMode.Brake);
        mElbowMotor.setNeutralMode(NeutralMode.Brake);

        mElbowMotor.setInverted(true);
        mShoulderMotor.setInverted(true);

        resetLash(correctedShoulderCanCoderPostion, correctedElbowCanCoderPostion);

        mMechanismActualRoot = mMechanismActual.getRoot("ArmRoot", Constants.Hand.maxFrameExtension.x, 0);
        mMechanismActualShoulder = mMechanismActualRoot.append(
                new
                 MechanismLigament2d("Shoulder", Constants.Arm.upperarmLength, 0, 3, new Color8Bit(Color.kGreen)));
        mMechanismActualElbow = mMechanismActualShoulder.append(
                new MechanismLigament2d("Elbow", Constants.Arm.forearmLength, 0, 3, new Color8Bit(Color.kGreen)));

        mMechanismTargetRoot = mMechanismTarget.getRoot("ArmRoot", Constants.Hand.maxFrameExtension.x, 0);
        mMechanismTargetShoulder = mMechanismTargetRoot.append(
                new MechanismLigament2d("Shoulder", Constants.Arm.upperarmLength, 0, 2, new Color8Bit(Color.kRed)));
        mMechanismTargetElbow = mMechanismTargetShoulder.append(
                new MechanismLigament2d("Elbow", Constants.Arm.forearmLength, 0, 2, new Color8Bit(Color.kRed)));

        mMechanismProfileRoot = mMechanismProfile.getRoot("ArmRoot", Constants.Hand.maxFrameExtension.x, 0);
        mMechanismProfileShoulder = mMechanismProfileRoot.append(
                new MechanismLigament2d("Shoulder", Constants.Arm.upperarmLength, 0, 2, new Color8Bit(Color.kHotPink)));
        mMechanismProfileElbow = mMechanismProfileShoulder.append(
                new MechanismLigament2d("Elbow", Constants.Arm.forearmLength, 0, 2, new Color8Bit(Color.kHotPink)));

        mMechanismMotorRoot = mMechanismMotor.getRoot("ArmRoot", Constants.Hand.maxFrameExtension.x, 0);
        mMechanismMotorShoulder = mMechanismMotorRoot.append(
                new MechanismLigament2d("Shoulder", Constants.Arm.upperarmLength, 0, 2, new Color8Bit(Color.kWhite)));
        mMechanismMotorElbow = mMechanismMotorShoulder.append(
                new MechanismLigament2d("Elbow", Constants.Arm.forearmLength, 0, 2, new Color8Bit(Color.kWhite)));

    }

    // Not needed if we use normal pid for predefined positions
    public static Utils.Vector2D calculateArmAngles(Utils.Vector2D targetPos) {
        double targetDist = Math.sqrt(targetPos.x * targetPos.x + targetPos.y * targetPos.y);
        if (targetDist > Constants.Arm.forearmLength + Constants.Arm.upperarmLength - 0.05) {
            double targetAngle = Math.atan2(targetPos.y, targetPos.x);
            targetPos.y = Math.sin(targetAngle) * (Constants.Arm.forearmLength + Constants.Arm.upperarmLength - 0.05);
            targetPos.x = Math.cos(targetAngle) * (Constants.Arm.forearmLength + Constants.Arm.upperarmLength - 0.05);
            DataLogManager.log("SETPOINT LIMITED BY ARM REACH");
        }
        if (Math.abs(targetPos.x) > Constants.Hand.maxFrameExtension.x) {
            targetPos.x = Math.signum(targetPos.x) * Constants.Hand.maxFrameExtension.x;
            DataLogManager.log("SETPOINT LIMITED BY X LEGAL LIMIT");

        }

        if (targetPos.y > Constants.Hand.maxFrameExtension.y) {
            targetPos.y = Constants.Hand.maxFrameExtension.y;
            DataLogManager.log("SETPOINT LIMITED BY Y LEGAL LIMIT");

        }
        if (targetPos.y < Constants.Arm.floorHeight) {
            targetPos.y = Constants.Arm.floorHeight;
            DataLogManager.log("SETPOINT LIMITED BY FLOOR");

        }

        double q2 = -Math.acos((Math.pow(targetPos.x, 2) + Math.pow(targetPos.y, 2)
                - Math.pow(Constants.Arm.upperarmLength, 2) - Math.pow(Constants.Arm.forearmLength, 2))
                / (2 * Constants.Arm.forearmLength * Constants.Arm.upperarmLength));
        double q1 = Math.atan2(targetPos.y, targetPos.x) - Math.atan2(Constants.Arm.forearmLength * Math.sin(q2),
                Constants.Arm.upperarmLength + Constants.Arm.forearmLength * Math.cos(q2));

        double correctedElbowAngle = Math.toDegrees(q2);
        double correctedShoulderAngle = Math.toDegrees(q1) ;

        //Go to tpose position instead of NaN when reaching inside itself
        if(Double.isNaN(correctedElbowAngle)){
            correctedElbowAngle = -90;
        }
        if(Double.isNaN(correctedShoulderAngle)){
            correctedShoulderAngle = 90;
        }

        return new Vector2D(correctedElbowAngle, correctedShoulderAngle);
        // return new Utils.Vector2D(Math.toDegrees(q2), Math.toDegrees(q1));
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
        double shoulderAngleActual = mShoulderCanCoder.getPosition();
        double elbowAngleActual = mElbowCanCoder.getPosition();

        Utils.Vector2D elbowPeakOutputs = new Utils.Vector2D(1.0, 1.0);
        Utils.Vector2D shoulderPeakOutputs = new Utils.Vector2D(1.0, 1.0);

        /*
         * Utils.Vector2D motorAngles = calculateArmAngles(mCurrentSetpoint);
         * 
         * //motorAngles.x += mYOffset;
         * //motorAngles.y += mXOffset;
         * 
         * shoulderPeakOutputs.x = 1.00;
         * shoulderPeakOutputs.y = 1.00;
         * 
         * elbowPeakOutputs.x = 1.00;
         * elbowPeakOutputs.y = 1.00;
         * 
         * double elbowError = motorAngles.x * 4096 / 360 -
         * mElbowMotor.getSelectedSensorPosition();
         * double shoulderError = motorAngles.y * 4096 / 360 -
         * mShoulderMotor.getSelectedSensorPosition();
         * 
         * if (mCurrentSetpoint != null) {
         * if(motorAngles.x != mLastMotorAngles.x){
         * mElbowHysteresis.reset();
         * }
         * if(motorAngles.y != mLastMotorAngles.y){
         * mShoulderHysteresis.reset();
         * }
         * if (Math.abs(elbowError) < Constants.Arm.noReduceThreshold) {
         * elbowPeakOutputs.y = 0.12 ;
         * }
         * if (Math.abs(elbowError) < Constants.Arm.noReduceThreshold
         * &&
         * mElbowDebouncer.calculate(mElbowHysteresis.calculate(Math.abs(elbowError))))
         * {
         * mElbowMotor.set(ControlMode.PercentOutput, 0);
         * mElbowBrakeSolenoid.set(false);
         * } else {
         * mElbowBrakeSolenoid.set(true);
         * mElbowMotor.set(ControlMode.MotionMagic, motorAngles.x * 4096 / 360 );
         * }
         * 
         * if (Math.abs(shoulderError) < Constants.Arm.noReduceThreshold) {
         * shoulderPeakOutputs.y = 0.1;
         * }
         * if ((Math.abs(shoulderError) < Constants.Arm.noReduceThreshold
         * && mShoulderDebouncer.calculate(mShoulderHysteresis.calculate(Math.abs(
         * shoulderError)))
         * )) {
         * mShoulderMotor.set(ControlMode.PercentOutput, 0);
         * mShoulderBrakeSolenoid.set(false);
         * } else {
         * mShoulderBrakeSolenoid.set(true);
         * mShoulderMotor.set(ControlMode.MotionMagic, motorAngles.y * 4096 / 360);
         * }
         */
        
        if (shoulderAngleActual > Constants.Arm.shoulderAngleForwardSoftStop) {
            shoulderPeakOutputs.x = 0.00;
            shoulderPeakOutputs.y = 1.00;
            SmartDashboard.putString("Shoulder/Soft Limit", "Forward (Up)");

        } else if (shoulderAngleActual < Constants.Arm.shoulderAngleReverseSoftStop) {
            shoulderPeakOutputs.x = 1.00;
            shoulderPeakOutputs.y = 0.00;
            SmartDashboard.putString("Shoulder/Soft Limit", "Reverse (Down)");

        }else{
            SmartDashboard.putString("Shoulder/Soft Limit", "None");
        }

        if (elbowAngleActual > Constants.Arm.elbowAngleForwardSoftStop) {
            elbowPeakOutputs.x = 0.00;
            elbowPeakOutputs.y = 1.00;
            SmartDashboard.putString("Elbow/Soft Limit", "Forward (Up)");

        } else if (elbowAngleActual < Constants.Arm.elbowAngleReverseSoftStop) {
            elbowPeakOutputs.x = 1.00;
            elbowPeakOutputs.y = 0.00;
            SmartDashboard.putString("Elbow/Soft Limit", "Reverse (Down)");
        }else{
            SmartDashboard.putString("Elbow/Soft Limit", "None");
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

        handPos = calculateHandPosition(new Utils.Vector2D(elbowAngleActual, shoulderAngleActual));
         SmartDashboard.putNumber("Hand/Actual X", handPos.x);
         SmartDashboard.putNumber("Hand/Actual Y", handPos.y);
        handPosFromMotors  = calculateHandPosition(new Utils.Vector2D(getElbowPositionFromMotor(), getShoulderPositionFromMotor()));
         SmartDashboard.putNumber("Hand/Motor X", handPosFromMotors.x);
         SmartDashboard.putNumber("Hand/Motor Y", handPosFromMotors.y);;

        // SmartDashboard.putNumber("SetPoint Hand X", mCurrentSetpoint.x);
        // SmartDashboard.putNumber("SetPoint Hand Y", mCurrentSetpoint.y);
        // mLastMotorAngles = motorAngles.clone();

        SmartDashboard.putBoolean("Elbow/Lock", getElbowLocked());
        SmartDashboard.putBoolean("Shoulder/Lock", getShoulderLocked());
        SmartDashboard.putData("ArmCommand",Arm.getInstance());

        // SmartDashboard.putNumber("Shoulder Cancoder", mShoulderCanCoder.getPosition());
        // SmartDashboard.putNumber("Shoulder Motor Encoder",
        //         ((mShoulderMotor.getSelectedSensorPosition() *Constants.Arm.);
        // SmartDashboard.putNumber("Elbow Cancoder", mElbowCanCoder.getPosition());
        // SmartDashboard.putNumber("Elbow Motor Encoder",
        //         -((mElbowMotor.getSelectedSensorPosition() / 4096) * 360 / Constants.Arm.elbowGearRatio));

        mMechanismActualElbow.setAngle(elbowAngleActual);
        mMechanismActualShoulder.setAngle(shoulderAngleActual);

        mMechanismMotorElbow.setAngle(getElbowPositionFromMotor());
        mMechanismMotorShoulder.setAngle(mShoulderMotor.getSelectedSensorPosition()*Constants.Arm.shoulderDegreesPerMotorTick);

        SmartDashboard.putData("ArmMechanism/Actual", mMechanismActual);
        SmartDashboard.putData("ArmMechanism/Target", mMechanismTarget);
        SmartDashboard.putData("ArmMechanism/Profile", mMechanismProfile);
        SmartDashboard.putData("ArmMechanism/Motor", mMechanismMotor);
    }

    public double getHandPositionX() {
        return handPosFromMotors.x;
    }

    // public void changeSetpoint(Utils.Vector2D currentAngles, Utils.Vector2D setpoint) {
    //     Vector2D mCurrentAngle = currentAngles;
    //     Vector2D mSetpoint = setpoint;

    //     if (mSetpoint != null && mCurrentAngle != null) {
    //         if (Math.abs(mSetpoint.x - mCurrentAngle.x) > (50/4096)*360) {
    //             mElbowBrakeSolenoid.set(true);
    //             mElbowMotor.set(ControlMode.MotionMagic, mSetpoint.x);
    //         }

    //         if (Math.abs(mSetpoint.y - mCurrentAngle.y) > (50/4096)*360) {
    //             mShoulderBrakeSolenoid.set(true);
    //             mShoulderMotor.set(ControlMode.MotionMagic, mSetpoint.y);
    //         }

    //         if (mElbowHysteresis.calculate(mElbowMotor.getClosedLoopError())) {
    //             mElbowBrakeSolenoid.set(false);
    //         }

    //         if (mShoulderHysteresis.calculate(mShoulderMotor.getClosedLoopError())) {
    //             mShoulderBrakeSolenoid.set(false);
    //         }
    //     }
    // }

    //Returns elbow motor angle in motor space with joint units
    public double setElbowPosition(Vector2D positions) {
        mMechanismTargetElbow.setAngle(positions.x);
        double decorrectedElbowAngle=decorrectElbowAngle(positions);
        setElbowPosition(decorrectedElbowAngle);
        return decorrectedElbowAngle;
    }

    //JUST CORRECTS FOR DEGREES PER MOTOR TICK, NOT COORDINATE SPACE CORRECTION
    public void setElbowPosition(double elbowPosition){
        setElbowLock(false);
        mElbowMotor.set(ControlMode.MotionMagic, elbowPosition/Constants.Arm.elbowDegreesPerMotorTick);
    }


    //Takes motor/target joint angles and returns elbow motor angle (where 0 is only parallel to forearm when vertical)
    public static double decorrectElbowAngle(Vector2D motorAnglesInJointSpace){
        return (motorAnglesInJointSpace.x - (90 - motorAnglesInJointSpace.y) / Constants.Arm.elbowVirtualFourBarRatio);
    }
    //Takes motor/target joint angles and returns elbow motor angle (where 0 is only parallel to forearm when vertical)
    public static double decorrectElbowVelocity(Vector2D motorVelocitiesInJointSpace){
        return (motorVelocitiesInJointSpace.x - motorVelocitiesInJointSpace.y / Constants.Arm.elbowVirtualFourBarRatio);
    }

     //Takes elbow motor angle (where 0 is only parallel to forearm when vertical) and shoulder angle and returns motor/target elbow angle 
     public double correctElbowAngle(Vector2D motorAnglesInMotorSpace){
        return (motorAnglesInMotorSpace.x + (90 - motorAnglesInMotorSpace.y) / Constants.Arm.elbowVirtualFourBarRatio);
    }


    public void setShoulderPosition(double shoulderPosition) {
        setShoulderLock(false);
        mMechanismTargetShoulder.setAngle(shoulderPosition);
        mShoulderMotor.set(ControlMode.MotionMagic, shoulderPosition/Constants.Arm.shoulderDegreesPerMotorTick);
    }

    public void updateTargetMechanism(Vector2D armAngles){
        mMechanismTargetElbow.setAngle(armAngles.getElbow());
        mMechanismTargetShoulder.setAngle(armAngles.getShoulder());
    }
    public void updateProfileMechanism(Vector2D armAngles){
        mMechanismProfileElbow.setAngle(armAngles.getElbow());
        mMechanismProfileShoulder.setAngle(armAngles.getShoulder());
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

    public void setPower(double shoulder, double elbow) {

        mShoulderMotor.set(ControlMode.PercentOutput, shoulder);
        mElbowMotor.set(ControlMode.PercentOutput, elbow);

        setShoulderLock(Math.abs(shoulder)<0.01);
        setElbowLock(Math.abs(elbow)<0.01);
    }

    public boolean getElbowLocked() {
        return !mElbowBrakeSolenoid.get();
    }

    public boolean getShoulderLocked() {
        return !mShoulderBrakeSolenoid.get();
    }

    //Returns elbow motor angle in joint units 
    public double getUncorrectedElbowMotorPosition(){
        return mElbowMotor.getSelectedSensorPosition()*Constants.Arm.elbowDegreesPerMotorTick;
    }
    public double getElbowPositionFromMotor(){
        double shoulderAngle = mShoulderMotor.getSelectedSensorPosition() * Constants.Arm.shoulderDegreesPerMotorTick;
        return correctElbowAngle(new Vector2D(getUncorrectedElbowMotorPosition(),shoulderAngle));
    }

    public void setShoulderLock(boolean value) {
        SmartDashboard.putBoolean("Shoulder/RequestedLock", value);
        mShoulderBrakeSolenoid.set(!value);
    }

    public void setElbowLock(boolean value) {
        SmartDashboard.putBoolean("Elbow/RequestedLock", value);
        mElbowBrakeSolenoid.set(!value);
    }

    public double getShoulderPositionFromMotor() {
        return mShoulderMotor.getSelectedSensorPosition() * Constants.Arm.shoulderDegreesPerMotorTick;
    }
    // public void setLight(double input) {
    // mBlinken.set(input);
    // }

    public void setElbowPower(double power) {
        mElbowMotor.set(ControlMode.PercentOutput,power);
    }
    public void setShoulderPower(double power) {
        mShoulderMotor.set(ControlMode.PercentOutput,power);
    }
    //TODO: WE "ARE BAD" AT SCHEDULING
//THESE THINGS ARE ALL BAD BECAUSE OF INIT RACE CONDITION
    public void resetLash(double correctedShoulderCanCoderPostion, double correctedElbowCanCoderPostion) {
        mShoulderMotor.setSelectedSensorPosition(-correctedShoulderCanCoderPostion/Constants.Arm.shoulderDegreesPerMotorTick);
        mElbowMotor.setSelectedSensorPosition(
                                                -decorrectElbowAngle(new Vector2D(correctedElbowCanCoderPostion,correctedShoulderCanCoderPostion))
                                                / Constants.Arm.elbowDegreesPerMotorTick
                                             );
        DataLogManager.log("Reset Lash");
    }
//TODO: THESE THINGS ARE ALL BAD BECAUSE OF INIT RACE CONDITION

    public void resetLash() {
        mShoulderMotor.setSelectedSensorPosition(mShoulderCanCoder.getPosition()/Constants.Arm.shoulderDegreesPerMotorTick);
        mElbowMotor.setSelectedSensorPosition(
                                                decorrectElbowAngle(new Vector2D(mElbowCanCoder.getPosition(),mShoulderCanCoder.getPosition()))
                                                / Constants.Arm.elbowDegreesPerMotorTick
                                             );
        DataLogManager.log("Reset Lash");
    }

    public ControlMode getElbowControlMode() {
        return mElbowMotor.getControlMode();
    }
    public ControlMode getShoulderControlMode() {
        return mShoulderMotor.getControlMode();
    }

    public void startMotionProfiles(BufferedTrajectoryPointStream elbowStream,BufferedTrajectoryPointStream shoulderStream){
        mElbowMotor.startMotionProfile(elbowStream,10,ControlMode.MotionProfile);//10 points to start with
        mShoulderMotor.startMotionProfile(shoulderStream,10,ControlMode.MotionProfile);//10 points to start with
    }
    public MotionProfileStatus getElbowMotionProfileStatus(){
        MotionProfileStatus status = new MotionProfileStatus();
        mElbowMotor.getMotionProfileStatus(status);
        return status;
    }
    
    public MotionProfileStatus getShoulderMotionProfileStatus(){
        MotionProfileStatus status = new MotionProfileStatus();
        mShoulderMotor.getMotionProfileStatus(status);
        return status;
    }
    public boolean getElbowMotionProfileFinished(){
        return mElbowMotor.isMotionProfileFinished();
    }
    public boolean getShoulderMotionProfileFinished(){
        return mShoulderMotor.isMotionProfileFinished();
    }
    public Vector2D getActiveTrajectoryArmAngles(){
        Vector2D ret = new Vector2D(mElbowMotor.getActiveTrajectoryPosition()*Constants.Arm.elbowDegreesPerMotorTick,mShoulderMotor.getActiveTrajectoryPosition()*Constants.Arm.shoulderDegreesPerMotorTick);
        ret.setElbow(correctElbowAngle(ret));
        return ret;
    }
    public Vector2D getActiveTrajectoryAngularVelocity(){
        Vector2D ret = new Vector2D(mElbowMotor.getActiveTrajectoryVelocity()*Constants.Arm.elbowDegreesPerMotorTick*10,mShoulderMotor.getActiveTrajectoryVelocity()*Constants.Arm.shoulderDegreesPerMotorTick*10);
        return ret;
    }

    public Vector2D getHandPosFromMotor() {
        return handPosFromMotors;
    }

    public double getElbowOutputPercent() {
        return mElbowMotor.getMotorOutputPercent();
    }public double getShoulderOutputPercent() {
        return mShoulderMotor.getMotorOutputPercent();
    }

    public Vector2D getMotorAngularVelocities() {
        Vector2D ret = new Vector2D(mElbowMotor.getSelectedSensorVelocity()*Constants.Arm.elbowDegreesPerMotorTick*10,mShoulderMotor.getSelectedSensorVelocity()*Constants.Arm.shoulderDegreesPerMotorTick*10);
        return ret;
        }
}
