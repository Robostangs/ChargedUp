package frc.ArmTrajectoryPlanner;

import frc.robot.Constants;
import frc.robot.Utils.Vector2D;

public class ArmTrajectoryPlannerTest {
    public static void main(String[] args){
        Thread thread= new Thread(()->{
            Vector2D maxSpeeds=new Vector2D(Constants.Arm.elbowCruiseVelocity,Constants.Arm.shoulderCruiseVelocity);
            maxSpeeds.multiply(10);//to units per second
            maxSpeeds.elementwiseMultiply(Constants.Arm.elbowDegreesPerMotorTick,Constants.Arm.shoulderDegreesPerMotorTick);//to degrees per second

            Vector2D maxAccels=new Vector2D(Constants.Arm.elbowAccelerationFactor,Constants.Arm.elbowAccelerationFactor);
            maxAccels.multiply(10);//to units per second squared
            maxAccels.elementwiseMultiply(Constants.Arm.elbowDegreesPerMotorTick,Constants.Arm.shoulderDegreesPerMotorTick);//to degrees per second squared

            ArmTrajectoryPlanner planner = new ArmTrajectoryPlanner(new Vector2D(1.44, 1.3), new Vector2D(0.27, 0.18), maxSpeeds, maxAccels, 1, 5, 0.01);
            planner.plan();
            planner.simulateToLog();
        });
        thread.start();
        
    }
}
