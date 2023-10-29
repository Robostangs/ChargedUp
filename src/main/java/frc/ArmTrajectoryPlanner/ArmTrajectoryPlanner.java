package frc.ArmTrajectoryPlanner;

import java.util.ArrayList;
import java.util.EnumSet;
import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.robot.Utils.Vector2D;
import frc.robot.subsystems.Arm;

public class ArmTrajectoryPlanner {

    /**
     * 
     * @param startPoint
     * @param endPoint
     * @param maxJointSpeeds       degrees/second
     * @param maxJointAcceleration degrees/second/second
     * @param targetSpeed          meters/second
     */

    public static final double sampleTime = 0.02;
    private static final boolean isDebug = false;
    private PathPoint startPoint, endPoint;
    private Vector2D maxJointSpeeds, maxJointAccelerations;
    private double targetMaxSpeed, originalTargetMaxSpeed, originalTargetMaxPosAccel, originalTargetMaxNegAccel,
            targetMaxPosAccel, targetMaxNegAccel;
    private ArrayList<Vector2D> pathPoints, pathVelocities, pathAccelerations, pathAngles, pathAngularVelocities,
            pathAngularAccelerations;
    private ArrayList<Double> timestamps, pathSplineAcceleration, pathSplineVelocity;
    public BufferedTrajectoryPointStream elbowTrajectoryPointStream = new BufferedTrajectoryPointStream();
    public BufferedTrajectoryPointStream shoulderTrajectoryPointStream = new BufferedTrajectoryPointStream();

    public ArmTrajectoryPlanner(PathPoint startPoint, PathPoint endPoint, double targetMaxSpeed,
            double targetMaxPosAccel, double targetMaxNegAccel) {
        if(isDebug){
            DriverStation.reportError(
                "#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@\nError: ARM TRAJECTORY PLANNER DEBUG MODE\n#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@",false);
                DriverStation.reportError(
                    "#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@\nError: ARM TRAJECTORY PLANNER DEBUG MODE\n#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@",false);
                    DriverStation.reportError(
                        "#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@\nError: ARM TRAJECTORY PLANNER DEBUG MODE\n#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@",false);
                        DriverStation.reportError(
                            "#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@\nError: ARM TRAJECTORY PLANNER DEBUG MODE\n#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@",false);
        }
        this.maxJointSpeeds = new Vector2D(Constants.Arm.elbowCruiseVelocity, Constants.Arm.shoulderCruiseVelocity);

        maxJointSpeeds.multiply(10);// to units per second
        maxJointSpeeds.elementwiseMultiply(Constants.Arm.elbowDegreesPerMotorTick,
                Constants.Arm.shoulderDegreesPerMotorTick);// to degrees per second

        this.maxJointAccelerations = new Vector2D(Constants.Arm.elbowAccelerationFactor,
                Constants.Arm.shoulderAccelerationFactor);

        maxJointAccelerations.multiply(10);// to units per second squared
        maxJointAccelerations.elementwiseMultiply(Constants.Arm.elbowDegreesPerMotorTick,
                Constants.Arm.shoulderDegreesPerMotorTick);// to degrees per second squared

        this.targetMaxSpeed = targetMaxSpeed;
        this.originalTargetMaxSpeed = this.targetMaxSpeed;
        this.targetMaxPosAccel = targetMaxPosAccel;
        this.targetMaxNegAccel = targetMaxNegAccel;

        this.originalTargetMaxPosAccel = this.targetMaxPosAccel;
        this.originalTargetMaxNegAccel = this.targetMaxNegAccel;

        this.startPoint = startPoint;
        this.endPoint = endPoint;
        
    }

    public void plan() {
        if (isDebug) {
            pathPoints = new ArrayList<>();
            pathVelocities = new ArrayList<>();
            pathSplineVelocity = new ArrayList<>();
            pathAccelerations = new ArrayList<>();
            pathSplineAcceleration = new ArrayList<>();
            pathAngularAccelerations = new ArrayList<>();
        }
        pathAngles = new ArrayList<>();
        pathAngularVelocities = new ArrayList<>();
        timestamps = new ArrayList<>();


        EnumSet<Arm.ViolationType> startPointViolations = Arm.checkArmPointViolation(new Vector2D(startPoint.position), EnumSet.noneOf(Arm.ViolationType.class)).violation;
        if(startPointViolations.contains(Arm.ViolationType.X_LEGAL)){
            startPoint=startPoint.withNewHeading(Rotation2d.fromDegrees(-180));
            DataLogManager.log("UPDATED START HEADING TO -180 FOR X LEGAL LIMIT");
        }

        if(startPointViolations.contains(Arm.ViolationType.Y_LEGAL)){
            startPoint=startPoint.withNewHeading(Rotation2d.fromDegrees(-90));
            DataLogManager.log("UPDATED START HEADING TO -90 FOR Y LEGAL LIMIT");
        }
        if(startPointViolations.contains(Arm.ViolationType.FLOOR)){
            startPoint=startPoint.withNewHeading(Rotation2d.fromDegrees(90));
            DataLogManager.log("UPDATED START HEADING TO 90 FOR FLOOR");
        }


        // needs to be here for speed/accel foldback regeneration
        PathPlannerTrajectory path = PathPlanner.generatePath(
                new PathConstraints(targetMaxSpeed, targetMaxPosAccel, targetMaxNegAccel),
                startPoint,
                endPoint);
        Vector2D startV2D = new Vector2D(startPoint.position);
        Vector2D endV2D = new Vector2D(endPoint.position);
        SmartDashboard.putNumber("ArmPlanner/startPoint/x", startPoint.position.getX());
        SmartDashboard.putNumber("ArmPlanner/startPoint/y", startPoint.position.getY());
        SmartDashboard.putNumber("ArmPlanner/startPoint/angle", startPoint.heading.getDegrees());
        SmartDashboard.putString("ArmPlanner/startPoint/name", startPoint.name);
        SmartDashboard.putNumber("ArmPlanner/endPoint/x", endPoint.position.getX());
        SmartDashboard.putNumber("ArmPlanner/endPoint/y", endPoint.position.getY());
        SmartDashboard.putNumber("ArmPlanner/endPoint/angle", endPoint.heading.getDegrees());
        SmartDashboard.putString("ArmPlanner/endPoint/name", endPoint.name);


        if (isDebug) {
            pathPoints.add(startV2D);
            pathVelocities.add(new Vector2D());
            pathSplineVelocity.add(0.0);
            pathAccelerations.add(new Vector2D());
            pathSplineAcceleration.add(0.0);
            pathAngularAccelerations.add(new Vector2D());
        }

        pathAngularVelocities.add(new Vector2D());
        timestamps.add(0.0);
        Vector2D lastAngles = Arm.calculateArmAnglesUnsafe(startV2D);
        pathAngles.add(lastAngles.clone());//NOT lastAngles cuz pass by reference

        Vector2D lastAngularVelocities = new Vector2D();
        Vector2D lastAngularAccelerations = new Vector2D();
        Vector2D lastVelocity = new Vector2D();

        for (double t = sampleTime; t < path.getTotalTimeSeconds(); t += sampleTime) {
            timestamps.add(t);

            PathPlannerState state = (PathPlannerState) path.sample(t);
            Vector2D currentVelocity, currentAcceleration;
            if (isDebug) {
                currentVelocity = new Vector2D(
                        state.poseMeters.getRotation().getCos() * state.velocityMetersPerSecond,
                        state.poseMeters.getRotation().getSin() * state.velocityMetersPerSecond);
                currentAcceleration = currentVelocity.getSubtracted(lastVelocity).getDivided(sampleTime);
                currentAcceleration = currentAcceleration.getMultiplied(0.3)
                        .getAdded(currentAcceleration.getMultiplied(1 - 0.3));// only used for display
            }

            Vector2D currentPosition = new Vector2D(state.poseMeters.getX(), state.poseMeters.getY());

            // filter angle and angular velocity together so they are synchronized
            Vector2D currentArmAngles = Arm.calculateArmAnglesWithDisabledViolations(currentPosition, startPointViolations);
            currentArmAngles.exponentialFilter(lastAngles, 0.3);
            Vector2D currentAngularVelocities = currentArmAngles.getDerivative(lastAngles,sampleTime);
            // currentAngularVelocities = currentAngularVelocities.getMultiplied(0.3)
            // .getAdded(lastAngularVelocities.getMultiplied(1 - 0.3));
            Vector2D currentAnglularAccelerations = currentAngularVelocities.getDerivative(lastAngularVelocities,sampleTime);// actually sent to motor, don't want too much latency
            currentAnglularAccelerations.exponentialFilter(lastAngularAccelerations, 0.05);// only used for checking limits, want more filter
            
            if(isDebug){
                pathPoints.add(currentPosition);
                pathVelocities.add(currentVelocity);
                pathSplineVelocity.add(state.velocityMetersPerSecond);
                pathAccelerations.add(currentAcceleration);
                pathSplineAcceleration.add(state.accelerationMetersPerSecondSq);
                pathAngularAccelerations.add(currentAnglularAccelerations);
            }
            pathAngles.add(currentArmAngles);
            pathAngularVelocities.add(currentAngularVelocities);

            boolean isSteadyState = state.accelerationMetersPerSecondSq < 0.1 * targetMaxPosAccel
                    && state.accelerationMetersPerSecondSq > -0.1 * targetMaxNegAccel;// not accelerating, must be angle
                                                                                      // weirdness
            if (Math.abs(currentAnglularAccelerations.getElbow()) > maxJointAccelerations.getElbow()) {
                if (isSteadyState) {
                    velocityFoldbackReplan("Elbow Acceleration");
                    return;
                } else {
                    if (state.accelerationMetersPerSecondSq > 0)
                        positiveAccelerationFoldbackReplan("Elbow Acceleration");
                    else
                        negativeAccelerationFoldbackReplan("Elbow Acceleration");
                    return;
                }
            }
            if (Math.abs(currentAngularVelocities.getElbow()) > maxJointSpeeds.getElbow()) {
                velocityFoldbackReplan("Elbow Velocity");
                return;
            }

            if (Math.abs(currentAnglularAccelerations.getShoulder()) > maxJointAccelerations.getShoulder()) {
                if (isSteadyState) {
                    velocityFoldbackReplan("Shoulder Acceleration");
                    return;
                } else {
                    if (state.accelerationMetersPerSecondSq > 0)
                        positiveAccelerationFoldbackReplan("Shoulder Acceleration");
                    else
                        negativeAccelerationFoldbackReplan("Shoulder Acceleration");
                    return;
                }
            }
            if (Math.abs(currentAngularVelocities.getShoulder()) > maxJointSpeeds.getShoulder()) {
                velocityFoldbackReplan("Shoulder Velocity");
                return;
            }

            lastAngles.set(currentArmAngles);
            lastAngularVelocities.set(currentAngularVelocities);
            lastAngularAccelerations.set(currentAnglularAccelerations);
            if(isDebug){
                lastVelocity.set(currentVelocity);
            }
        }

        pathAngles.add(Arm.calculateArmAngles(endV2D));
        pathAngularVelocities.add(new Vector2D());
        timestamps.add(timestamps.get(timestamps.size() - 1) + sampleTime);

        if (isDebug) {
            pathPoints.add(endV2D);
            pathVelocities.add(new Vector2D());
            pathSplineVelocity.add(0.0);
            pathAccelerations.add(new Vector2D());
            pathSplineAcceleration.add(0.0);
            pathAngularAccelerations.add(new Vector2D());
            SmartDashboard.putNumberArray("ArmPlanner/pathPointsDisplay/x",
                    pathPoints.stream().mapToDouble((vector) -> {
                        return Constants.Hand.maxFrameExtension.x + vector.x;
                    }).toArray());
            SmartDashboard.putNumberArray("ArmPlanner/pathPointsDisplay/y",
                    pathPoints.stream().mapToDouble((vector) -> {
                        return Constants.Hand.maxFrameExtension.y - vector.y;
                    }).toArray());
            String csvHeader = String.format(
                    "%-8s, %-8s, %-8s, %-8s, %-8s, %-8s, %-8s, %-8s, %-8s, %-8s, %-8s, %-8s, %-8s, %-8s, %-8s",
                    "Time", "VelSplin", "AccSplin",
                    "Pos X",
                    "Pos Y", "Vel X", "Vel Y", "Acc X", "Acc Y", "Elb Ang", "Sho Ang", "Elb Vel", "Sho Vel",
                    "Elb Acc", "Sho Acc");

            dumpCsvToConsole(csvHeader, timestamps, pathSplineVelocity, pathSplineAcceleration, pathPoints,
                    pathVelocities, pathAccelerations, pathAngles,
                    pathAngularVelocities,
                    pathAngularAccelerations);
        }

        buildFalconProfiles();
        DataLogManager.log("~~~~~~~~~~~~~~~Arm Path Planned~~~~~~~~~~~~~~~~~~");
    }

    private void buildFalconProfiles() {
        elbowTrajectoryPointStream = new BufferedTrajectoryPointStream();
        shoulderTrajectoryPointStream = new BufferedTrajectoryPointStream();
        for (int i = 0; i < timestamps.size(); i++) {
            TrajectoryPoint elbowPoint = new TrajectoryPoint();
            elbowPoint.position = Arm.decorrectElbowAngle(pathAngles.get(i)) / Constants.Arm.elbowDegreesPerMotorTick;
            elbowPoint.velocity = Arm.decorrectElbowVelocity(pathAngularVelocities.get(i))
                    / Constants.Arm.elbowDegreesPerMotorTick / 10;
            elbowPoint.arbFeedFwd = 0;
            elbowPoint.profileSlotSelect0 = 0;
            elbowPoint.isLastPoint = (i == timestamps.size() - 1);
            elbowPoint.zeroPos = false;// Don't zero the sensor
            elbowPoint.timeDur = 0;// Don't wait longer than is set in config
            elbowTrajectoryPointStream.Write(elbowPoint);

            TrajectoryPoint shoulderPoint = new TrajectoryPoint();
            shoulderPoint.position = pathAngles.get(i).getShoulder() / Constants.Arm.shoulderDegreesPerMotorTick;
            shoulderPoint.velocity = pathAngularVelocities.get(i).getShoulder()
                    / Constants.Arm.shoulderDegreesPerMotorTick / 10;
            shoulderPoint.arbFeedFwd = 0;
            shoulderPoint.profileSlotSelect0 = 0;
            shoulderPoint.isLastPoint = (i == timestamps.size() - 1);
            shoulderPoint.zeroPos = false;// Don't zero the sensor
            shoulderPoint.timeDur = 0;// Don't wait longer than is set in config
            shoulderTrajectoryPointStream.Write(shoulderPoint);
        }
    }

    @SafeVarargs
    private void dumpCsvToConsole(String heading, ArrayList<Double> timestamps, ArrayList<Double> pathVel,
            ArrayList<Double> pathAcc, ArrayList<Vector2D>... data) {
        StringBuilder builder = new StringBuilder("~~~~~~~~~~~~~~~Arm Path Planned~~~~~~~~~~~~~~~~~~").append("\n")
                .append(heading).append("\n");
        for (int i = 0; i < timestamps.size(); i++) {
            builder.append(String.format("%8.3f, ", timestamps.get(i)));
            builder.append(String.format("%8.3f, ", pathVel.get(i)));
            builder.append(String.format("%8.3f, ", pathAcc.get(i)));

            for (ArrayList<Vector2D> list : data) {
                builder.append(String.format("%8.3f, ", list.get(i).x));
                builder.append(String.format("%8.3f, ", list.get(i).y));
            }
            builder.append("\n");
        }
        builder.append("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
        DataLogManager.log(builder.toString());
    }

    private void velocityFoldbackReplan(String source) {
        targetMaxSpeed *= 0.8;
        if (targetMaxSpeed > 0.1 * originalTargetMaxSpeed) {
            DataLogManager
                    .log("Warning: ArmTrajectoryPlanner Backed off to " + (targetMaxSpeed / originalTargetMaxSpeed)
                            + "x original target speed due to " + source);
            plan();
        } else {
            DataLogManager.log("Error: ArmTrajectoryPlanner Cannot generate path with acceptable" + source);
        }
    }

    private void positiveAccelerationFoldbackReplan(String source) {
        targetMaxPosAccel *= 0.8;
        if (targetMaxPosAccel > 0.1 * originalTargetMaxPosAccel) {
            DataLogManager
                    .log("Warning: ArmTrajectoryPlanner Backed off to "
                            + (targetMaxPosAccel / originalTargetMaxPosAccel)
                            + "x original target positive accel due to " + source);
            plan();
        } else {
            DataLogManager.log("Error: ArmTrajectoryPlanner Cannot generate path with acceptable" + source);
        }
    }

    private void negativeAccelerationFoldbackReplan(String source) {
        targetMaxNegAccel *= 0.8;
        if (targetMaxNegAccel > 0.1 * originalTargetMaxNegAccel) {
            DataLogManager
                    .log("Warning: ArmTrajectoryPlanner Backed off to "
                            + (targetMaxNegAccel / originalTargetMaxNegAccel)
                            + "x original target neagtive accel due to " + source);
            plan();
        } else {
            DataLogManager.log("Error: ArmTrajectoryPlanner Cannot generate path with acceptable" + source);
        }
    }

    public void simulateToLogInOtherThread() {
        new Thread(() -> {
            simulateToLog();
        }).start();
    }

    private void simulateToLog() {
        Mechanism2d mMechanismTarget = new Mechanism2d(Constants.Hand.maxFrameExtension.x * 2,
                Constants.Hand.maxFrameExtension.y);
        MechanismRoot2d mMechanismTargetRoot;
        MechanismLigament2d mMechanismTargetShoulder;
        MechanismLigament2d mMechanismTargetElbow;
        mMechanismTargetRoot = mMechanismTarget.getRoot("ArmRoot", Constants.Hand.maxFrameExtension.x, 0);
        mMechanismTargetShoulder = mMechanismTargetRoot.append(
                new MechanismLigament2d("Shoulder", Constants.Arm.upperarmLength, 0, 2, new Color8Bit(Color.kRed)));
        mMechanismTargetElbow = mMechanismTargetShoulder.append(
                new MechanismLigament2d("Elbow", Constants.Arm.forearmLength, 0, 2, new Color8Bit(Color.kRed)));
        for (int i = 0; i < timestamps.size(); i++) {
            mMechanismTargetElbow.setAngle(pathAngles.get(i).getElbow());
            mMechanismTargetShoulder.setAngle(pathAngles.get(i).getShoulder());
            SmartDashboard.putData("sim arm", mMechanismTarget);
            try {
                Thread.sleep((long) (sampleTime * 1000));
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

}
