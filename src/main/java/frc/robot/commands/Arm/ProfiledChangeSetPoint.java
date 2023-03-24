package frc.robot.commands.Arm;

import javax.xml.crypto.Data;

import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.ArmTrajectoryPlanner.ArmTrajectoryPlanner;
import frc.robot.Constants;
import frc.robot.Utils;
import frc.robot.Utils.LockHysteresis;
import frc.robot.Utils.Vector2D;
import frc.robot.subsystems.Arm;

public class ProfiledChangeSetPoint extends CommandBase {

    private Arm mArm = Arm.getInstance();
    private final XboxController mManipController = new XboxController(1);
    private final Debouncer leftButtonDebouncer = new Debouncer(0.1, DebounceType.kRising);
    private final Debouncer rightButtonDebouncer = new Debouncer(0.1, DebounceType.kRising);
    private ArmTrajectoryPlanner mPlanner;
    private Vector2D mSetPoint, mSetPointInAngles;
    private double uncorrectedElbowTarget;
    LockHysteresis mElbowHysteresis = new LockHysteresis(Constants.Arm.elbowLockThreshold, Constants.Arm.elbowLockThreshold * 2);
    LockHysteresis mShoulderHysteresis = new LockHysteresis(Constants.Arm.shoulderLockThreshold, Constants.Arm.shoulderLockThreshold * 2);

    private ProfiledChangeSetPoint(PathPoint startPoint, PathPoint endPoint, double targetMaxSpeed, double targetMaxAccel) {
        addRequirements(mArm);
        setName("ProfiledChangeSetPoint");
        mPlanner = new ArmTrajectoryPlanner(startPoint, endPoint, targetMaxSpeed, targetMaxAccel);
        mSetPoint = new Vector2D(endPoint.position);
    }

    @Override
    public void initialize() {
        DataLogManager.log("Setpoint: " + mSetPoint.x + "," + mSetPoint.y);
        long preplanTime = System.nanoTime();
        mPlanner.plan();
        DataLogManager.log("Trajectory planning took "+((double)(System.nanoTime()-preplanTime)/1000000)+"ms");
        mPlanner.simulateToLogInOtherThread();
        SmartDashboard.putNumber("Hand Target X", mSetPoint.x);
        SmartDashboard.putNumber("Hand Target Y", mSetPoint.y);

        mSetPointInAngles = Arm.calculateArmAngles(mSetPoint);
        mArm.updateTargetMechanism(mSetPointInAngles);
        uncorrectedElbowTarget = Arm.decorrectElbowAngle(mSetPointInAngles);
        mArm.setElbowLock(false);
        mArm.setShoulderLock(false);
        mArm.startMotionProfiles(mPlanner.elbowTrajectoryPointStream, mPlanner.shoulderTrajectoryPointStream);
        mElbowHysteresis.reset();
        mShoulderHysteresis.reset();
    }

    // @Override
    // public void initSendable(SendableBuilder builder) {
    //     System.out.println("I HAVE COMPLETED THE INFINITY GAUNTLET");
    // }

    @Override
    public void execute() {
        SmartDashboard.putNumber("Elbow PID Error",mSetPointInAngles.x - mArm.getElbowPositionFromMotor());
        SmartDashboard.putNumber("Shoulder PID Error",mSetPointInAngles.y - mArm.getShoulderPositionFromMotor());

        if(mElbowHysteresis.calculate(Math.abs(uncorrectedElbowTarget - mArm.getUncorrectedElbowMotorPosition()))&&mArm.getElbowMotionProfileFinished()) {
            mArm.setElbowLock(true);
            mArm.setElbowPower(0);
        }
        if(mShoulderHysteresis.calculate(Math.abs(mSetPointInAngles.y - mArm.getShoulderPositionFromMotor()))&&mArm.getShoulderMotionProfileFinished()) {
            mArm.setShoulderLock(true);
            mArm.setShoulderPower(0);
        }
    }

    @Override
    public boolean isFinished() {
        if(mElbowHysteresis.get()&&mArm.getElbowMotionProfileFinished() && mShoulderHysteresis.get()&&mArm.getShoulderMotionProfileFinished()) {
            return true;
        }
        if((Math.abs(Utils.customDeadzone(mManipController.getLeftY())) > 0.1 || 
            Math.abs(Utils.customDeadzone(mManipController.getRightY())) > 0.1) && 
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
            DataLogManager.log("Profiled Change Set Point interrupted");
        }
    }

    public static ParallelRaceGroup createWithTimeout(PathPoint startPoint, PathPoint endPoint, double targetMaxSpeed, double targetMaxAccel, double timeout) {
        return new ProfiledChangeSetPoint(startPoint, endPoint, targetMaxSpeed, targetMaxAccel).withTimeout(timeout);
    }
    
    public static ParallelRaceGroup createWithTimeout(PathPoint startPoint, PathPoint endPoint) {
        return createWithTimeout(startPoint, endPoint, 3, 2,3);
    }
}
