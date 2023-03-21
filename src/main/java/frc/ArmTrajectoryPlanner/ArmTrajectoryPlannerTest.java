package frc.ArmTrajectoryPlanner;

import frc.robot.Utils.Vector2D;

public class ArmTrajectoryPlannerTest {
    public static void main(String[] args){
        ArmTrajectoryPlanner planner = new ArmTrajectoryPlanner(new Vector2D(1,0), new Vector2D(1,1), null, null, 1, 0, 0, 5, 0.01);
        planner.plan();
    
    }
}
