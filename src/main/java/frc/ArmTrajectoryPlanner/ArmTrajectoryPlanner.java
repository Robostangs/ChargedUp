package frc.ArmTrajectoryPlanner;

import java.util.ArrayList;

import frc.robot.Utils.Vector2D;

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
        System.out.println("t,X,Y,Vx,Vy,Ax,Ay");
        //plan forward
        for (double t=0;t<10;t+=sampleTime) {
            Vector2D lastPosition=pathPoints.get(pathPoints.size()-1);
            Vector2D lastVelocity=pathVelocities.get(pathVelocities.size()-1);
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
            pathPoints.add(currentPosition);
            pathVelocities.add(currentVelocity);
            pathAccelerations.add(currentAcceleration);
            System.out.println(t+","+currentPosition.x+","+currentPosition.y+","+currentVelocity.x+","+currentVelocity.y+","+currentAcceleration.x+","+currentAcceleration.y);
        }
        pathPoints.add(endPoint);
        pathVelocities.add(new Vector2D());
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
    

}
