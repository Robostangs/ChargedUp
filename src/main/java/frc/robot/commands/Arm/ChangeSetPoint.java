package frc.robot.commands.Arm;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Utils;
import frc.robot.Utils.LockHysteresis;
import frc.robot.Utils.Vector2D;
import frc.robot.subsystems.Arm;

public class ChangeSetPoint extends CommandBase {

    private Arm mArm = Arm.getInstance();
    private Vector2D mSetPoint, mSetPointInAngles;
    public static Vector2D mCurrentSetpoint = new Vector2D(0.4, 0.1);


    LockHysteresis mElbowHysteresis = new LockHysteresis(Constants.Arm.elbowLockThreshold, Constants.Arm.elbowLockThreshold * 8);
    LockHysteresis mShoulderHysteresis = new LockHysteresis(Constants.Arm.shoulderLockThreshold, Constants.Arm.shoulderLockThreshold * 5);

    public ChangeSetPoint(Vector2D setPoint) {
        mSetPoint = setPoint;
        addRequirements(mArm);
        setName("ChangeSetPoint");
    }

    @Override
    public void initialize() {
        DataLogManager.log("Setpoint: " + mSetPoint.x + "," + mSetPoint.y);


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
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        if(!interrupted) {
            mArm.setElbowLock(true);
            mArm.setShoulderLock(true);
        }
    }



}
