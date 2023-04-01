package frc.robot.commands.Arm;

import javax.xml.crypto.Data;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Constants;
import frc.robot.Utils;
import frc.robot.Utils.LockHysteresis;
import frc.robot.Utils.Vector2D;
import frc.robot.subsystems.Arm;

public class ChangeSetPoint extends CommandBase {

    private Arm mArm = Arm.getInstance();
    private Vector2D mSetPoint, mSetPointInAngles;
    private double uncorrectedElbowTarget;
    public static Vector2D mCurrentSetpoint = new Vector2D(0.4, 0.1);
    private final XboxController mManipController = new XboxController(1);
    private final Debouncer leftButtonDebouncer = new Debouncer(0.1, DebounceType.kRising);
    private final Debouncer rightButtonDebouncer = new Debouncer(0.1, DebounceType.kRising);

    LockHysteresis mElbowHysteresis = new LockHysteresis(Constants.Arm.elbowLockThreshold, Constants.Arm.elbowLockThreshold * 2);
    LockHysteresis mShoulderHysteresis = new LockHysteresis(Constants.Arm.shoulderLockThreshold, Constants.Arm.shoulderLockThreshold * 2);

    private ChangeSetPoint(Vector2D setPoint) {
        mSetPoint = setPoint;
        addRequirements(mArm);
        setName("ChangeSetPoint");
    }

    @Override
    public void initialize() {
        DataLogManager.log("Setpoint: " + mSetPoint.x + "," + mSetPoint.y);
        SmartDashboard.putNumber("Hand/Target X", mSetPoint.x);
        SmartDashboard.putNumber("Hand/Target Y", mSetPoint.y);


        mSetPointInAngles = mArm.calculateArmAngles(mSetPoint);

        uncorrectedElbowTarget=mArm.setElbowPosition(mSetPointInAngles);
        mArm.setShoulderPosition(mSetPointInAngles.y);

        mCurrentSetpoint.set(mSetPoint);

        mElbowHysteresis.reset();
        mShoulderHysteresis.reset();
    }

    // @Override
    // public void initSendable(SendableBuilder builder) {
    //     System.out.println("I HAVE COMPLETED THE INFINITY GAUNTLET");
    // }

    @Override
    public void execute() {
        SmartDashboard.putNumber("Elbow/PID Error",uncorrectedElbowTarget - mArm.getUncorrectedElbowMotorPosition());
        SmartDashboard.putNumber("Shoulder/PID Error",mSetPointInAngles.y - mArm.getShoulderPositionFromMotor());

        if(mElbowHysteresis.calculate(Math.abs(uncorrectedElbowTarget - mArm.getUncorrectedElbowMotorPosition()))) {
            mArm.setElbowLock(true);
            mArm.setElbowPower(0);
        }else{
            if(mArm.getElbowControlMode()==ControlMode.PercentOutput)
                mArm.setElbowPosition(mSetPointInAngles);
            mArm.setElbowLock(false);
        }
        if(mShoulderHysteresis.calculate(Math.abs(mSetPointInAngles.y - mArm.getShoulderPositionFromMotor()))) {
            mArm.setShoulderLock(true);
            mArm.setShoulderPower(0);
        }else{
            if(mArm.getShoulderControlMode()==ControlMode.PercentOutput)
                mArm.setShoulderPosition(mSetPointInAngles.y);
            mArm.setShoulderLock(false);
        }
    }

    @Override
    public boolean isFinished() {
        if(mElbowHysteresis.get() && mShoulderHysteresis.get()) {
            return true;
        }
        if((Math.abs(Utils.customDeadzone(mManipController.getLeftY())) > 0.1 || 
            Math.abs(Utils.customDeadzone(mManipController.getRightY())) > 0.1) && 
            leftButtonDebouncer.calculate(!mManipController.getLeftStickButton()) && 
            rightButtonDebouncer.calculate(!mManipController.getRightStickButton())
          ) {
            DataLogManager.log("Change Set Point interrupted");
            return true;
        }
        if((Math.abs(Utils.customDeadzone(mManipController.getLeftY())) > 0 || 
            Math.abs(Utils.customDeadzone(mManipController.getRightY())) > 0) && 
            leftButtonDebouncer.calculate(!mManipController.getLeftStickButton()) && 
            rightButtonDebouncer.calculate(!mManipController.getRightStickButton())
          ) {
            end(true);
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        mArm.setElbowLock(true);
        mArm.setShoulderLock(true);
        mArm.setShoulderPower(0);
        mArm.setElbowPower(0);
        if(interrupted) {
        }
    }

    public static ParallelRaceGroup createWithTimeout(Vector2D setPoint, double timeout) {
        return new ChangeSetPoint(setPoint).withTimeout(timeout);
    }
    
    public static ParallelRaceGroup createWithTimeout(Vector2D setPoint) {
        return createWithTimeout(setPoint, 3);
    }
}
