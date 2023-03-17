package frc.robot.commands.Arm;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Constants;
import frc.robot.Utils;
import frc.robot.Utils.LockHysteresis;
import frc.robot.Utils.Vector2D;
import frc.robot.subsystems.Arm;

public class ChangeSetPoint extends CommandBase {

    private Arm mArm = Arm.getInstance();
    private Vector2D mSetPoint, mSetPointInAngles;
    public static Vector2D mCurrentSetpoint = new Vector2D(0.4, 0.1);
    private final XboxController mManipController = new XboxController(1);
    private final Debouncer leftButtonDebouncer = new Debouncer(0.1, DebounceType.kRising);
    private final Debouncer rightButtonDebouncer = new Debouncer(0.1, DebounceType.kRising);

    LockHysteresis mElbowHysteresis = new LockHysteresis(Constants.Arm.elbowLockThreshold, Constants.Arm.elbowLockThreshold * 8);
    LockHysteresis mShoulderHysteresis = new LockHysteresis(Constants.Arm.shoulderLockThreshold, Constants.Arm.shoulderLockThreshold * 5);

    private ChangeSetPoint(Vector2D setPoint) {
        mSetPoint = setPoint;
        addRequirements(mArm);
        setName("ChangeSetPoint");
    }

    @Override
    public void initialize() {
        DataLogManager.log("Setpoint: " + mSetPoint.x + "," + mSetPoint.y);
        SmartDashboard.putNumber("Hand Target X", mSetPoint.x);
        SmartDashboard.putNumber("Hand Target Y", mSetPoint.y);


        mSetPointInAngles = mArm.calculateArmAngles(mSetPoint);

        mArm.setElbowPosition(mSetPointInAngles);
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
        SmartDashboard.putNumber("Elbow PID Error",mSetPointInAngles.x - mArm.getElbowPositionFromMotor());
        SmartDashboard.putNumber("Shoulder PID Error",mSetPointInAngles.y - mArm.getShoulderPositionFromMotor());

        if(mElbowHysteresis.calculate(Math.abs(mSetPointInAngles.x - mArm.getElbowPositionFromMotor()))) {
            mArm.setElbowLock(true);
            mArm.setElbowPower(0);
        }else{
            mArm.setElbowLock(false);
        }
        if(mShoulderHysteresis.calculate(Math.abs(mSetPointInAngles.y - mArm.getShoulderPositionFromMotor()))) {
            mArm.setShoulderLock(true);
            mArm.setShoulderPower(0);
        }else{
            mArm.setShoulderLock(false);
        }
    }

    @Override
    public boolean isFinished() {
        if(mElbowHysteresis.get() && mShoulderHysteresis.get()) {
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
            DataLogManager.log("Change Set Point interrupted");
        }
    }

    public static ParallelRaceGroup createWithTimeout(Vector2D setPoint, double timeout) {
        return new ChangeSetPoint(setPoint).withTimeout(timeout);
    }
    
    public static ParallelRaceGroup createWithTimeout(Vector2D setPoint) {
        return createWithTimeout(setPoint, 3);
    }
}
