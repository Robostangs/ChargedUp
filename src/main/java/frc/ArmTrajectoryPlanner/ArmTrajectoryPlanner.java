package frc.ArmTrajectoryPlanner;

import java.util.ArrayList;
import java.util.Collection;
import java.util.StringJoiner;

import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.TimeSyncEventData;
import edu.wpi.first.wpilibj.DataLogManager;
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

    public static final double sampleTime=0.01;
    private PathPoint startPoint,endPoint;
    private Vector2D maxJointSpeeds, maxJointAccelerations;
    private double targetMaxSpeed, originalTargetMaxSpeed, originalTargetMaxAccel, targetMaxAccel;
    private ArrayList<Vector2D> pathPoints, pathVelocities, pathAccelerations, pathAngles, pathAngularVelocities,
            pathAngularAccelerations;
    private ArrayList<Double> timestamps;
    public BufferedTrajectoryPointStream elbowTrajectoryPointStream = new BufferedTrajectoryPointStream();
    public BufferedTrajectoryPointStream shoulderTrajectoryPointStream = new BufferedTrajectoryPointStream();


    public ArmTrajectoryPlanner(PathPoint startPoint, PathPoint endPoint, double targetMaxSpeed, double targetMaxAccel) {
        this.maxJointSpeeds = new Vector2D(Constants.Arm.elbowCruiseVelocity,Constants.Arm.shoulderCruiseVelocity);

        maxJointSpeeds.multiply(10);//to units per second
        maxJointSpeeds.elementwiseMultiply(Constants.Arm.elbowDegreesPerMotorTick,Constants.Arm.shoulderDegreesPerMotorTick);//to degrees per second

        this.maxJointAccelerations = new Vector2D(Constants.Arm.elbowAccelerationFactor,Constants.Arm.shoulderAccelerationFactor);

        maxJointAccelerations.multiply(10);//to units per second squared
        maxJointAccelerations.elementwiseMultiply(Constants.Arm.elbowDegreesPerMotorTick,Constants.Arm.shoulderDegreesPerMotorTick);//to degrees per second squared

        this.targetMaxSpeed = targetMaxSpeed;
        this.originalTargetMaxSpeed = this.targetMaxSpeed;
        this.targetMaxAccel = targetMaxAccel;
        this.originalTargetMaxAccel = this.targetMaxAccel;
        this.startPoint = startPoint;
        this.endPoint = endPoint;
    }

    public void plan() {
        pathPoints = new ArrayList<>();
        pathVelocities = new ArrayList<>();
        pathAccelerations = new ArrayList<>();
        pathAngles = new ArrayList<>();
        pathAngularVelocities = new ArrayList<>();
        pathAngularAccelerations = new ArrayList<>();
        timestamps = new ArrayList<>();
        //needs to be here for speed/accel foldback regeneration
        PathPlannerTrajectory path = PathPlanner.generatePath(
            new PathConstraints(targetMaxSpeed, targetMaxAccel),
            startPoint,
            endPoint);
        Vector2D startV2D = new Vector2D(startPoint.position);
        Vector2D endV2D = new Vector2D(endPoint.position);

        pathPoints.add(startV2D);
        pathVelocities.add(new Vector2D());
        pathAccelerations.add(new Vector2D());
        pathAngles.add(Arm.calculateArmAngles(startV2D));
        pathAngularVelocities.add(new Vector2D());
        pathAngularAccelerations.add(new Vector2D());
        timestamps.add(0.0);

        for (double t = sampleTime; t < 10; t += sampleTime) {
            Vector2D lastAngles = pathAngles.get(pathAngles.size() - 1);
            Vector2D lastAngularVelocities = pathAngularVelocities.get(pathAngularVelocities.size() - 1);
            Vector2D lastAngularAcceleration = pathAngularAccelerations.get(pathAngularAccelerations.size() - 1);
            Vector2D lastVelocity = pathVelocities.get(pathVelocities.size() - 1);
            timestamps.add(t);
            if (t > path.getTotalTimeSeconds())
                break;
            PathPlannerState state = (PathPlannerState) path.sample(t);

            // Vector2D currentAcceleration = new
            // Vector2D(state.poseMeters.getRotation().getCos()*state.accelerationMetersPerSecondSq,state.poseMeters.getRotation().getSin()*state.accelerationMetersPerSecondSq);
            Vector2D currentVelocity = new Vector2D(
                    state.poseMeters.getRotation().getCos() * state.velocityMetersPerSecond,
                    state.poseMeters.getRotation().getSin() * state.velocityMetersPerSecond);
            Vector2D currentPosition = new Vector2D(state.poseMeters.getX(), state.poseMeters.getY());
            Vector2D currentAcceleration = currentVelocity.getSubtracted(lastVelocity).getDivided(sampleTime);
            currentAcceleration = currentAcceleration.getMultiplied(0.3)
                    .getAdded(currentAcceleration.getMultiplied(1 - 0.3));// only used for display

            // filter angle and angular velocity together so they are synchronized
            Vector2D currentArmAngles = Arm.calculateArmAngles(currentPosition);
            currentArmAngles = currentArmAngles.getMultiplied(0.3).getAdded(lastAngles.getMultiplied(1 - 0.3));
            Vector2D currentAngularVelocities = currentArmAngles.getSubtracted(lastAngles).getDivided(sampleTime);
            currentAngularVelocities = currentAngularVelocities.getMultiplied(0.3)
                    .getAdded(lastAngularVelocities.getMultiplied(1 - 0.3));
            Vector2D currentAnglularAccelerations = currentAngularVelocities.getSubtracted(lastAngularVelocities)
                    .getDivided(sampleTime);// actually sent to motor, don't want too much latency
            currentAnglularAccelerations = currentAnglularAccelerations.getMultiplied(0.05)
                    .getAdded(lastAngularAcceleration.getMultiplied(1 - 0.05));// only used for checking limits, want
                                                                              // more filter

            pathPoints.add(currentPosition);
            pathVelocities.add(currentVelocity);
            pathAccelerations.add(currentAcceleration);
            pathAngles.add(currentArmAngles);
            pathAngularVelocities.add(currentAngularVelocities);
            pathAngularAccelerations.add(currentAnglularAccelerations);

            boolean isSteadyState = state.accelerationMetersPerSecondSq < 0.1 * targetMaxAccel;// not accelerating, must be
                                                                                           // angle weirdness
            if (Math.abs(currentAnglularAccelerations.getElbow()) > maxJointAccelerations.getElbow()) {
                if (isSteadyState) {
                    velocityFoldbackReplan("Elbow Acceleration");
                    return;
                } else {
                    accelerationFoldbackReplan("Elbow Acceleration");
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
                    accelerationFoldbackReplan("Shoulder Acceleration");
                    return;
                }
            }
            if (Math.abs(currentAngularVelocities.getShoulder()) > maxJointSpeeds.getShoulder()) {
                velocityFoldbackReplan("Shoulder Velocity");
                return;
            }

            // System.out.printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
            // t,
            // currentPosition.x, currentPosition.y, currentVelocity.x, currentVelocity.y,
            // currentAcceleration.x,
            // currentAcceleration.y, currentArmAngles.x, currentArmAngles.y,
            // currentAngularVelocities.x,
            // currentAngularVelocities.y, currentAnglularAccelerations.x,
            // currentAnglularAccelerations.y);
            // System.out.printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
            // t,currentPosition.x,currentPosition.y, state.poseMeters.getX(),
            // state.poseMeters.getY(),currentVelocity.x,currentVelocity.y,state.poseMeters.getRotation().getCos()*state.velocityMetersPerSecond,state.poseMeters.getRotation().getSin()*state.velocityMetersPerSecond,currentAcceleration.x,
            // currentAcceleration.y,state.poseMeters.getRotation().getCos()*state.accelerationMetersPerSecondSq,state.poseMeters.getRotation().getSin()*state.accelerationMetersPerSecondSq);

            // System.out.println(t+","+currentPosition.x+","+currentPosition.y+","+currentVelocity.x+","+currentVelocity.y+","+currentAcceleration.x+","+currentAcceleration.y);
        }
        pathPoints.add(endV2D);
        pathVelocities.add(new Vector2D());
        pathAccelerations.add(new Vector2D());

        pathAngles.add(Arm.calculateArmAngles(endV2D));
        pathAngularVelocities.add(new Vector2D());
        pathAngularAccelerations.add(new Vector2D());
        timestamps.add(timestamps.get(timestamps.size() - 1) + sampleTime);
        SmartDashboard.putNumberArray("ArmPlanner/pathPointsDisplay/X", pathPoints.stream().mapToDouble((vector) -> {
            return Constants.Hand.maxFrameExtension.x + vector.x;
        }).toArray());
        SmartDashboard.putNumberArray("ArmPlanner/pathPointsDisplay/Y", pathPoints.stream().mapToDouble((vector) -> {
            return Constants.Hand.maxFrameExtension.y - vector.y;
        }).toArray());

        String csvHeader = String.format("%-8s, %-8s, %-8s, %-8s, %-8s, %-8s, %-8s, %-8s, %-8s, %-8s, %-8s, %-8s, %-8s",
                "Time",
                "Pos X",
                "Pos Y", "Vel X", "Vel Y", "Acc X", "Acc Y", "Elb Ang", "Sho Ang", "Elb Vel", "Sho Vel",
                "Elb Acc", "Sho Acc");
        dumpCsvToConsole(csvHeader, timestamps, pathPoints, pathVelocities, pathAccelerations, pathAngles,
                pathAngularVelocities,
                pathAngularAccelerations);

        buildFalconProfiles();
    }

    private void buildFalconProfiles() {
        elbowTrajectoryPointStream = new BufferedTrajectoryPointStream();
        shoulderTrajectoryPointStream = new BufferedTrajectoryPointStream();
        for (int i = 0; i < pathPoints.size(); i++) {
            TrajectoryPoint elbowPoint = new TrajectoryPoint();
            elbowPoint.position = Arm.decorrectElbowAngle(pathAngles.get(i))/Constants.Arm.elbowDegreesPerMotorTick;
            elbowPoint.velocity = pathAngularVelocities.get(i).getElbow()/Constants.Arm.elbowDegreesPerMotorTick/10;
            elbowPoint.arbFeedFwd = 0;
            elbowPoint.profileSlotSelect0 = 0;
            elbowPoint.isLastPoint = (i==pathPoints.size()-1);
            elbowPoint.zeroPos = false;//Don't zero the sensor
            elbowPoint.timeDur = 0;//Don't wait longer than is set in config
            elbowTrajectoryPointStream.Write(elbowPoint);

            TrajectoryPoint shoulderPoint = new TrajectoryPoint();
            elbowPoint.position = pathAngles.get(i).getShoulder()/Constants.Arm.shoulderDegreesPerMotorTick;
            elbowPoint.velocity = pathAngularVelocities.get(i).getShoulder()/Constants.Arm.shoulderDegreesPerMotorTick/10;
            elbowPoint.arbFeedFwd = 0;
            elbowPoint.profileSlotSelect0 = 0;
            elbowPoint.isLastPoint = (i==pathPoints.size()-1);
            elbowPoint.zeroPos = false;//Don't zero the sensor
            elbowPoint.timeDur = 0;//Don't wait longer than is set in config
            shoulderTrajectoryPointStream.Write(shoulderPoint);
        }
    }

    @SafeVarargs
    private void dumpCsvToConsole(String heading, ArrayList<Double> timestamps, ArrayList<Vector2D>... data) {
        StringBuilder builder = new StringBuilder("~~~~~~~~~~~~~~~Arm Path Planned~~~~~~~~~~~~~~~~~~").append("\n")
                .append(heading).append("\n");
        for (int i = 0; i < pathPoints.size(); i++) {
            builder.append(String.format("%8.3f, ", timestamps.get(i)));
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
        targetMaxSpeed *= 0.9;
        if (targetMaxSpeed > 0.1 * originalTargetMaxSpeed) {
            DataLogManager
                    .log("Warning: ArmTrajectoryPlanner Backed off to " + (targetMaxSpeed / originalTargetMaxSpeed)
                            + "x original target speed due to " + source);
            plan();
        } else {
            DataLogManager.log("Error: ArmTrajectoryPlanner Cannot generate path with acceptable" + source);
        }
    }

    private void accelerationFoldbackReplan(String source) {
        targetMaxAccel *= 0.9;
        if (targetMaxAccel > 0.1 * originalTargetMaxAccel) {
            DataLogManager
                    .log("Warning: ArmTrajectoryPlanner Backed off to " + (targetMaxAccel / originalTargetMaxAccel)
                            + "x original target accel due to " + source);
            plan();
        } else {
            DataLogManager.log("Error: ArmTrajectoryPlanner Cannot generate path with acceptable" + source);
        }
    }
    public void simulateToLogInOtherThread(){
        new Thread(()->{simulateToLog();}).start();
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
        for (int i = 0; i < pathPoints.size(); i++) {
            mMechanismTargetElbow.setAngle(pathAngles.get(i).getElbow());
            mMechanismTargetShoulder.setAngle(pathAngles.get(i).getShoulder());
            SmartDashboard.putData("sim arm", mMechanismTarget);
            try {
                Thread.sleep((long) (sampleTime * 1000));
            } catch (InterruptedException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }
        }
    }

}