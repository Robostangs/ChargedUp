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
    private double targetMaxSpeed, originalTargetMaxSpeed, originalTargetMaxAccel, targetMaxAccel, sampleTime;
    private ArrayList<Vector2D> pathPoints, pathVelocities, pathAccelerations, pathAngles, pathAngularVelocities, pathAngularAccelerations;
    
    public ArmTrajectoryPlanner(Vector2D startPoint, Vector2D endPoint, Vector2D maxJointSpeeds,
            Vector2D maxJointAccelerations, double targetMaxSpeed, double targetMaxAccel, double sampleTime) {
        this.startPoint = startPoint;
        this.endPoint = endPoint;
        this.maxJointSpeeds = maxJointSpeeds;
        this.maxJointAccelerations = maxJointAccelerations;
        this.targetMaxSpeed = targetMaxSpeed;
        this.originalTargetMaxSpeed=this.targetMaxSpeed;
        this.targetMaxAccel = targetMaxAccel;
        this.originalTargetMaxAccel=this.targetMaxAccel;
        this.sampleTime = sampleTime;
 
    }
    
    public void plan(){
        pathPoints=new ArrayList<>();
        pathVelocities=new ArrayList<>();
        pathAccelerations=new ArrayList<>();
        pathAngles=new ArrayList<>();
        pathAngularVelocities=new ArrayList<>();
        pathAngularAccelerations=new ArrayList<>();
        pathPoints.add(startPoint);
        pathVelocities.add(new Vector2D());
        pathAccelerations.add(new Vector2D());
        pathAngles.add(Arm.getInstance().calculateArmAngles(startPoint));
        pathAngularVelocities.add(new Vector2D());
        pathAngularAccelerations.add(new Vector2D());
        System.out.println("t,X,Y,Vx,Vy,Ax,Ay,E,S,Ve,Vs,Ae,As");
        
        PathPlannerTrajectory path = PathPlanner.generatePath(
            new PathConstraints(targetMaxSpeed, targetMaxAccel), 
            new PathPoint(new Translation2d(startPoint.x,startPoint.y),Rotation2d.fromDegrees(180)).withControlLengths(1, 1),
            new PathPoint(new Translation2d(endPoint.x,endPoint.y),Rotation2d.fromDegrees(180+45)).withControlLengths(0.25, 0.25));
        for (double t=0;t<10;t+=sampleTime) {
            Vector2D lastAngles=pathAngles.get(pathAngles.size()-1);
            Vector2D lastAngularVelocities=pathAngularVelocities.get(pathAngularVelocities.size()-1);
            Vector2D lastAngularAcceleration=pathAngularAccelerations.get(pathAngularAccelerations.size()-1);
            Vector2D lastVelocity=pathVelocities.get(pathVelocities.size()-1);

            if(t>path.getTotalTimeSeconds())
                break;
            PathPlannerState state = (PathPlannerState) path.sample(t);

            //Vector2D currentAcceleration = new Vector2D(state.poseMeters.getRotation().getCos()*state.accelerationMetersPerSecondSq,state.poseMeters.getRotation().getSin()*state.accelerationMetersPerSecondSq);
            Vector2D currentVelocity = new Vector2D(state.poseMeters.getRotation().getCos()*state.velocityMetersPerSecond,state.poseMeters.getRotation().getSin()*state.velocityMetersPerSecond);
            Vector2D currentPosition = new Vector2D(state.poseMeters.getX(),state.poseMeters.getY());
            Vector2D currentAcceleration = currentVelocity.getSubtracted(lastVelocity).getDivided(sampleTime);
            currentAcceleration = currentAcceleration.getMultiplied(0.3).getAdded(currentAcceleration.getMultiplied(1-0.3));// only used for display

            Vector2D currentArmAngles = Arm.getInstance().calculateArmAngles(currentPosition);
            Vector2D currentAngularVelocities = currentArmAngles.getSubtracted(lastAngles).getDivided(sampleTime);
            currentAngularVelocities = currentAngularVelocities.getMultiplied(0.3).getAdded(lastAngularVelocities.getMultiplied(1-0.3));
            Vector2D currentAnglularAccelerations = currentAngularVelocities.getSubtracted(lastAngularVelocities).getDivided(sampleTime);//actually sent to motor, don't want too much latency
            currentAnglularAccelerations = currentAnglularAccelerations.getMultiplied(0.1).getAdded(lastAngularAcceleration.getMultiplied(1-0.1));//only used for checking limits, want more filter



            pathPoints.add(currentPosition);
            pathVelocities.add(currentVelocity);
            pathAccelerations.add(currentAcceleration);
            pathAngles.add(currentArmAngles);
            pathAngularVelocities.add(currentAngularVelocities);
            pathAngularAccelerations.add(currentAnglularAccelerations);
            

            if(Math.abs(currentAnglularAccelerations.getElbow())>maxJointAccelerations.getElbow()){
               System.out.println("Elbow accel exceeded"); 
               if(currentAcceleration.getLength()<0.1){//not accelerating, must be angle weirdness
                    targetMaxSpeed*=0.9;
                    if(targetMaxSpeed>0.1*originalTargetMaxSpeed){
                        System.out.println("Backed off to "+(targetMaxSpeed/originalTargetMaxSpeed)+"x original target speed");
                        plan();
                        return;
                    }else{
                        System.err.println("Cannot generate path with acceptable elbow acceleration");
                    }
               }else{
                targetMaxAccel*=0.9;
                    if(targetMaxAccel>0.1*originalTargetMaxAccel){
                        System.out.println("Backed off to "+(targetMaxAccel/originalTargetMaxAccel)+"x original target accel");
                        plan();
                        return;
                    }else{
                        System.err.println("Cannot generate path with acceptable elbow acceleration");
                    }
               }
            }
            if(Math.abs(currentAngularVelocities.getElbow())>maxJointSpeeds.getElbow()){
                System.out.println("Elbow Angular Velocity Exceeded");
                targetMaxSpeed*=0.9;
                    if(targetMaxSpeed>0.1*originalTargetMaxSpeed){
                        System.out.println("Backed off to "+(targetMaxSpeed/originalTargetMaxSpeed)+"x original target speed");
                        plan();
                    }else{
                        System.err.println("Cannot generate path with acceptable elbow speed");
                    }
            }

            System.out.printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", t,currentPosition.x,currentPosition.y, currentVelocity.x,currentVelocity.y,currentAcceleration.x, currentAcceleration.y, currentArmAngles.x, currentArmAngles.y, currentAngularVelocities.x, currentAngularVelocities.y, currentAnglularAccelerations.x, currentAnglularAccelerations.y);
            //System.out.printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", t,currentPosition.x,currentPosition.y, state.poseMeters.getX(), state.poseMeters.getY(),currentVelocity.x,currentVelocity.y,state.poseMeters.getRotation().getCos()*state.velocityMetersPerSecond,state.poseMeters.getRotation().getSin()*state.velocityMetersPerSecond,currentAcceleration.x, currentAcceleration.y,state.poseMeters.getRotation().getCos()*state.accelerationMetersPerSecondSq,state.poseMeters.getRotation().getSin()*state.accelerationMetersPerSecondSq);

            //System.out.println(t+","+currentPosition.x+","+currentPosition.y+","+currentVelocity.x+","+currentVelocity.y+","+currentAcceleration.x+","+currentAcceleration.y);
        }
        pathPoints.add(endPoint);
        pathVelocities.add(new Vector2D());
        pathAngles.add(Arm.getInstance().calculateArmAngles(endPoint));
        pathAngularVelocities.add(new Vector2D());
        SmartDashboard.putNumberArray("pathPoints/X",pathPoints.stream().mapToDouble((vector)->{return Constants.Hand.maxFrameExtension.x+vector.x;}).toArray());
            SmartDashboard.putNumberArray("pathPoints/Y",pathPoints.stream().mapToDouble((vector)->{return Constants.Hand.maxFrameExtension.y-vector.y;}).toArray());
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
