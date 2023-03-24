package frc.ArmTrajectoryPlanner;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.Utils.Vector2D;
import com.pathplanner.lib.PathPoint;

public class ArmTrajectoryPlannerTest {
    public static void main(String[] args){
            ArmTrajectoryPlanner planner = new ArmTrajectoryPlanner(
                                                new PathPoint(new Translation2d(1.44, 1.3), Rotation2d.fromDegrees(180)).withControlLengths(1, 1),
                                                new PathPoint(new Translation2d(0.27, 0.18), Rotation2d.fromDegrees(180)).withControlLengths(1, 1),
                                                4,
                                                3);
            planner.plan();
            planner.simulateToLogInOtherThread();
        
    }
}
