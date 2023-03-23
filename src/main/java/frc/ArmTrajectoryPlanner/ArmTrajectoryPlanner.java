package frc.ArmTrajectoryPlanner;

import java.util.ArrayList;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
     * @param maxJointSpeeds degrees/second
     * @param maxJointAcceleration degrees/second/second
     * @param targetSpeed meters/second
     */

    private Vector2D startPoint, endPoint, maxJointSpeeds, maxJointAccelerations;
    private double targetMaxSpeed, targetStartSpeed, targetEndSpeed, targetMaxAccel, sampleTime;
    private ArrayList<Vector2D> pathPoints, pathVelocities, pathAccelerations, pathAngles, pathAngularVelocities, pathAngularAccelerations;
    
    public ArmTrajectoryPlanner(Vector2D startPoint, Vector2D endPoint, Vector2D maxJointSpeeds,
            Vector2D maxJointAccelerations, double targetMaxSpeed, double targetStartSpeed, double targetEndSpeed, double targetMaxAccel, double sampleTime) {
        this.startPoint = startPoint;
        this.endPoint = endPoint;
        this.maxJointSpeeds = maxJointSpeeds;
        this.maxJointAccelerations = maxJointAccelerations;
        this.targetMaxSpeed = targetMaxSpeed;
        this.targetStartSpeed = targetStartSpeed;
        this.targetEndSpeed = targetEndSpeed;
        this.targetMaxAccel = targetMaxAccel;
        this.sampleTime = sampleTime;
        pathPoints=new ArrayList<>();
        pathVelocities=new ArrayList<>();
        pathAccelerations=new ArrayList<>();
        pathAngles=new ArrayList<>();
        pathAngularVelocities=new ArrayList<>();
        pathAngularAccelerations=new ArrayList<>();
    }
    
    public void plan(){
        double maxAllowableSpeed = Math.sqrt(2.0*targetMaxAccel*startPoint.distance(endPoint));
        targetMaxSpeed = Math.min(targetMaxSpeed,maxAllowableSpeed);
        double accelerationDistance=targetMaxSpeed*targetMaxSpeed/((2.0*targetMaxAccel));
        Vector2D deltaVector=endPoint.getSubtracted(startPoint);
        pathPoints.add(startPoint);
        pathVelocities.add(new Vector2D());
        pathAccelerations.add(new Vector2D());
        pathAngles.add(Arm.getInstance().calculateArmAngles(startPoint));
        pathAngularVelocities.add(new Vector2D());
        pathAngularAccelerations.add(new Vector2D());
        System.out.println("t,X,Y,Vx,Vy,Ax,Ay,E,S,Ve,Vs,Ae,As");

        PathPlannerTrajectory path = PathPlanner.generatePath(
            new PathConstraints(targetMaxSpeed, targetMaxAccel), 
            new PathPoint(new Translation2d(startPoint.x,startPoint.y),Rotation2d.fromDegrees(0)),
            new PathPoint(new Translation2d(endPoint.x,endPoint.y),Rotation2d.fromDegrees(45)));
        for (double t=0;t<10;t+=sampleTime) {
            Vector2D lastPosition=pathPoints.get(pathPoints.size()-1);
            Vector2D lastVelocity=pathVelocities.get(pathVelocities.size()-1);
            Vector2D lastAngles=pathAngles.get(pathAngles.size()-1);
            Vector2D lastAngularVelocity=pathAngularVelocities.get(pathAngularVelocities.size()-1);
            double targetAcceleration=0;
            double currentSpeed = lastVelocity.getLength(); 
            if(lastPosition.distance(startPoint)<accelerationDistance){
                targetAcceleration=Math.min(targetMaxAccel,(targetMaxSpeed-currentSpeed)/sampleTime);
            }else if(lastPosition.distance(endPoint)<accelerationDistance){
                targetAcceleration=Math.max(-targetMaxAccel,-currentSpeed/sampleTime);
                if(currentSpeed<0.001){
                    break;
                }
            }
            Vector2D currentAcceleration = deltaVector.getNormalized().getMultiplied(targetAcceleration);
            Vector2D currentVelocity = lastVelocity.getAdded(currentAcceleration.getMultiplied(sampleTime));
            Vector2D currentPosition = lastPosition.getAdded(currentVelocity.getMultiplied(sampleTime));

            Vector2D currentArmAngles = Arm.getInstance().calculateArmAngles(currentPosition);
            Vector2D currentAngularVelocities = currentArmAngles.getSubtracted(lastAngles).getDivided(sampleTime);
            Vector2D currentAnglularAccelerations = currentAngularVelocities.getSubtracted(lastAngularVelocity).getDivided(sampleTime);


            pathPoints.add(currentPosition);
            pathVelocities.add(currentVelocity);
            pathAccelerations.add(currentAcceleration);
            pathAngles.add(currentArmAngles);
            pathAngularVelocities.add(currentAngularVelocities);
            pathAngularAccelerations.add(currentAnglularAccelerations);
            PathPlannerState state = (PathPlannerState) path.sample(t);
            //System.out.printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", t,currentPosition.x,currentPosition.y, currentVelocity.x,currentVelocity.y,currentAcceleration.x, currentAcceleration.y, currentArmAngles.x, currentArmAngles.y, currentAngularVelocities.x, currentAngularVelocities.y, currentAnglularAccelerations.x, currentAnglularAccelerations.y);
            System.out.printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", t,currentPosition.x,currentPosition.y, state.poseMeters.getX(), state.poseMeters.getY(),currentVelocity.x,currentVelocity.y,state.poseMeters.getRotation().getCos()*state.velocityMetersPerSecond,state.poseMeters.getRotation().getSin()*state.velocityMetersPerSecond,currentAcceleration.x, currentAcceleration.y,state.poseMeters.getRotation().getCos()*state.accelerationMetersPerSecondSq,state.poseMeters.getRotation().getSin()*state.accelerationMetersPerSecondSq);

            //System.out.println(t+","+currentPosition.x+","+currentPosition.y+","+currentVelocity.x+","+currentVelocity.y+","+currentAcceleration.x+","+currentAcceleration.y);
        }
        pathPoints.add(endPoint);
        pathVelocities.add(new Vector2D());
        pathAngles.add(Arm.getInstance().calculateArmAngles(endPoint));
        pathAngularVelocities.add(new Vector2D());
    }

    public Vector2D getStartPoint() {
        return startPoint;
    }

    public Vector2D getEndPoint() {
        return endPoint;
    }

    public Vector2D getMaxJointSpeeds() {
        return maxJointSpeeds;
    }

    public Vector2D getMaxJointAccelerations() {
        return maxJointAccelerations;
    }

    public double getTargetMaxSpeed() {
        return targetMaxSpeed;
    }

    public double getTargetStartSpeed() {
        return targetStartSpeed;
    }

    public double getTargetEndSpeed() {
        return targetEndSpeed;
    }

    public double getTargetMaxAccel() {
        return targetMaxAccel;
    }

    public double getSampleTime() {
        return sampleTime;
    }

    public ArrayList<Vector2D> getPathPoints() {
        return pathPoints;
    }

    public ArrayList<Vector2D> getPathAngles() {
        return pathAngles;
    }

    public ArrayList<Vector2D> getPathAngularVelocities() {
        return pathAngularVelocities;
    }

    public ArrayList<Vector2D> getPathVelocities() {
        return pathVelocities;
    }

    public ArrayList<Vector2D> getPathAccelerations() {
        return pathAccelerations;
    }

    public ArrayList<Vector2D> getPathAngularAccelerations() {
        return pathAngularAccelerations;
    }

    public void simulateToLog() {
         Mechanism2d mMechanismTarget = new Mechanism2d(Constants.Hand.maxFrameExtension.x*2, Constants.Hand.maxFrameExtension.y);
     MechanismRoot2d mMechanismTargetRoot;
     MechanismLigament2d mMechanismTargetShoulder;
     MechanismLigament2d mMechanismTargetElbow;
        mMechanismTargetRoot = mMechanismTarget.getRoot("ArmRoot", Constants.Hand.maxFrameExtension.x, 0);
        mMechanismTargetShoulder = mMechanismTargetRoot.append(
                new MechanismLigament2d("Shoulder", Constants.Arm.upperarmLength, 0, 2, new Color8Bit(Color.kRed)));
        mMechanismTargetElbow = mMechanismTargetShoulder.append(
                new MechanismLigament2d("Elbow", Constants.Arm.forearmLength, 0, 2, new Color8Bit(Color.kRed)));
        for(int i=0;i<pathPoints.size();i++){
            mMechanismTargetElbow.setAngle(pathAngles.get(i).getElbow());
            mMechanismTargetShoulder.setAngle(pathAngles.get(i).getShoulder());
            SmartDashboard.putData("sim arm",mMechanismTarget);
            try {
                Thread.sleep((long)(sampleTime*1000));
            } catch (InterruptedException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }
        }
    }
    

}
