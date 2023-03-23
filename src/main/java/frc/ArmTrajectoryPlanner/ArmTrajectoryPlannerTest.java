package frc.ArmTrajectoryPlanner;

import frc.robot.Utils.Vector2D;

public class ArmTrajectoryPlannerTest {
    public static void main(String[] args){
        Thread thread= new Thread(()->{
            ArmTrajectoryPlanner planner = new ArmTrajectoryPlanner(new Vector2D(1,0), new Vector2D(0,1), null, null, 1, 0, 0, 5, 0.01);
            planner.plan();
            planner.simulateToLog();
        });
        thread.start();
        
    }
}
