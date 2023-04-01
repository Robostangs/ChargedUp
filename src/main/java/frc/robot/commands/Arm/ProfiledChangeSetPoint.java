package frc.robot.commands.Arm;

import java.util.function.Supplier;

import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.ArmTrajectoryPlanner.ArmTrajectoryPlanner;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Utils;
import frc.robot.Utils.LockHysteresis;
import frc.robot.Utils.Vector2D;
import frc.robot.subsystems.Arm;

public class ProfiledChangeSetPoint extends CommandBase {

    private static Arm mArm = Arm.getInstance();
    private static final XboxController mManipController = new XboxController(1);
    private static final Debouncer leftButtonDebouncer = new Debouncer(0.1, DebounceType.kRising);
    private static final Debouncer rightButtonDebouncer = new Debouncer(0.1, DebounceType.kRising);
    private ArmTrajectoryPlanner mPlanner;
    private Vector2D mSetPoint, mSetPointInAngles;
    private double uncorrectedElbowTarget;
    LockHysteresis mElbowHysteresis = new LockHysteresis(Constants.Arm.elbowLockThreshold, Constants.Arm.elbowLockThreshold * 2);
    LockHysteresis mShoulderHysteresis = new LockHysteresis(Constants.Arm.shoulderLockThreshold, Constants.Arm.shoulderLockThreshold * 2);
    private Supplier<PathPoint> startPointSupplier;
    private Supplier<PathPoint> endPointSupplier;
    private double targetMaxSpeed;
    private double targetMaxPosAccel,targetMaxNegAccel;

    private ProfiledChangeSetPoint(Supplier<PathPoint> startPointSupplier, Supplier<PathPoint> endPointSupplier, double targetMaxSpeed, double targetMaxPosAccel, double targetMaxNegAccel) {
        this.startPointSupplier = startPointSupplier;
        this.endPointSupplier = endPointSupplier;
        this.targetMaxSpeed = targetMaxSpeed;
        this.targetMaxPosAccel = targetMaxPosAccel;
        this.targetMaxNegAccel = targetMaxNegAccel;
        addRequirements(mArm);
        setName("ProfiledChangeSetPoint");
    }

    @Override
    public void initialize() {
        PathPoint startPoint = startPointSupplier.get().flipHeading();
        PathPoint endPoint = endPointSupplier.get();
        if(endPoint.position.getDistance(startPoint.position)<0.2){
            this.cancel();
            ChangeSetPoint.createWithTimeout(new Vector2D(endPoint.position)).schedule();
            return;
        }
        mSetPoint = new Vector2D(endPoint.position);
        DataLogManager.log(String.format("ProfiledChangeSetPoint from (%3.3f,%3.3f)@%3.3f deg to (%3.3f,%3.3f)@%3.3f deg with velocity %3.3f, pos accel %3.3f, neg accel %3.3f",startPoint.position.getX(), startPoint.position.getY(),startPoint.heading.getDegrees(),endPoint.position.getX(), endPoint.position.getY(),endPoint.heading.getDegrees(), targetMaxSpeed, targetMaxPosAccel, targetMaxNegAccel));
        long preplanTime = System.nanoTime();
        mPlanner = new ArmTrajectoryPlanner(startPoint, endPoint, targetMaxSpeed, targetMaxPosAccel, targetMaxNegAccel);
        mPlanner.plan();
        SmartDashboard.putNumber("ArmPlanner/Planning Time (ms)",((double)(System.nanoTime()-preplanTime)/1000000));
        if(Robot.isSimulation()){
            mPlanner.simulateToLogInOtherThread();
        }

        SmartDashboard.putNumber("Hand/Target X", mSetPoint.x);
        SmartDashboard.putNumber("Hand/Target Y", mSetPoint.y);


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
        SmartDashboard.putNumber("Elbow/PID Error",mSetPointInAngles.x - mArm.getElbowPositionFromMotor());
        SmartDashboard.putNumber("Shoulder/PID Error",mSetPointInAngles.y - mArm.getShoulderPositionFromMotor());
        mArm.updateProfileMechanism(mArm.getActiveTrajectoryArmAngles());
        Vector2D activeTrajectoryAngularVelocity = mArm.getActiveTrajectoryAngularVelocity();
        SmartDashboard.putNumber("Elbow/Profile Velocity", activeTrajectoryAngularVelocity.getElbow());
        SmartDashboard.putNumber("Shoulder/Profile Velocity", activeTrajectoryAngularVelocity.getShoulder());

        SmartDashboard.putNumber("Elbow/Output Percent",mArm.getElbowOutputPercent());
        SmartDashboard.putNumber("Shoulder/Output Percent",mArm.getShoulderOutputPercent());
        Vector2D motorAngularVelocities = mArm.getMotorAngularVelocities();
        SmartDashboard.putNumber("Elbow/Current Velocity",motorAngularVelocities.getElbow());
        SmartDashboard.putNumber("Shoulder/Current Velocity",motorAngularVelocities.getShoulder());



        // if(mElbowHysteresis.calculate(Math.abs(uncorrectedElbowTarget - mArm.getUncorrectedElbowMotorPosition()))&&mArm.getElbowMotionProfileFinished()) {
        //     mArm.setElbowLock(true);
        //     mArm.setElbowPower(0);
        // }
        // if(mShoulderHysteresis.calculate(Math.abs(mSetPointInAngles.y - mArm.getShoulderPositionFromMotor()))&&mArm.getShoulderMotionProfileFinished()) {
        //     mArm.setShoulderLock(true);
        //     mArm.setShoulderPower(0);
        // }
    }

    @Override
    public boolean isFinished() {
        if(mElbowHysteresis.calculate(
            Math.abs(uncorrectedElbowTarget - mArm.getUncorrectedElbowMotorPosition()))
            && mArm.getElbowMotionProfileFinished() 
            && mShoulderHysteresis.calculate(Math.abs(mSetPointInAngles.y - mArm.getShoulderPositionFromMotor()))
            && mArm.getShoulderMotionProfileFinished()) {
            return true;
        }
        return false;
    }

    public static void laterEnd(boolean interrupted) {
        mArm.setElbowLock(true);
        mArm.setShoulderLock(true);
        mArm.setShoulderPower(0);
        mArm.setElbowPower(0);
        if (interrupted) {
            if ((Math.abs(Utils.customDeadzone(mManipController.getLeftY())) > 0.1 ||
                    Math.abs(Utils.customDeadzone(mManipController.getRightY())) > 0.1)) {
                DataLogManager.log("PCSP Probably Manual Override");
            }else{
                DataLogManager.log("PCSP Interrupted");
            }

        } else
            DataLogManager.log("PCSP Ended Normally");

    }

    public static Command createWithTimeout(Supplier<PathPoint> startPointSupplier, Supplier<PathPoint> endPointSupplier, double targetMaxSpeed, double targetMaxPosAccel, double targetMaxNegAccel, double timeout) {
        return new ProfiledChangeSetPoint(startPointSupplier, endPointSupplier, targetMaxSpeed, targetMaxPosAccel, targetMaxNegAccel)
        .withTimeout(timeout)
        .andThen(new WaitCommand(0.5))
        .until(()->{return(Math.abs(Utils.customDeadzone(mManipController.getLeftY())) > 0.1 || 
            Math.abs(Utils.customDeadzone(mManipController.getRightY())) > 0.1) && 
            leftButtonDebouncer.calculate(!mManipController.getLeftStickButton()) && 
            rightButtonDebouncer.calculate(!mManipController.getRightStickButton());})
        .finallyDo((interrupted)->ProfiledChangeSetPoint.laterEnd(interrupted))//after wait or interrupted
        .withName("ProfiledChangeSetPoint");
    }
    public static Command createWithTimeout(Supplier<PathPoint> startPointSupplier, Supplier<PathPoint> endPointSupplier) {
        return createWithTimeout(startPointSupplier, endPointSupplier, 7, 3,3,4);
    }
    public static Command createWithTimeout(Supplier<PathPoint> endPointSupplier) {
        return createWithTimeout(()->getClosestPathPoint(), endPointSupplier);
    }

    public static PathPoint getClosestPathPoint() {
        final PathPoint[] allPoints = 
          { Constants.Arm.SetPoint.upIntakePosition, 
            Constants.Arm.SetPoint.generalIntakePosition,
            Constants.Arm.SetPoint.coneHighPosition,
            Constants.Arm.SetPoint.coneMediumPosition,
            Constants.Arm.SetPoint.cubeHighPosition,
            Constants.Arm.SetPoint.cubeMediumPosition,
            Constants.Arm.SetPoint.lowPosition,
            Constants.Arm.SetPoint.loadingZonePosition,
            Constants.Arm.SetPoint.stowPosition,
            Constants.Arm.SetPoint.startPosition };

        Translation2d armTranslation = Arm.getInstance().getHandPosFromMotor().toTranslation2d();
        double smallestDistance = 100;
        PathPoint closestPathPoint = null;
        for(int i = 0; i < allPoints.length; i++) {
            double distance = armTranslation.getDistance(allPoints[i].position);
            if(distance < smallestDistance) {
                smallestDistance = distance;
                closestPathPoint = allPoints[i];
            }
        }
        
        return closestPathPoint.withNewTranslation(armTranslation);
    }
}
