package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class translatePpNotRelative {
            
    // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
    static SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        Swerve.getInstance()::getPose, // Pose2d supplier
        (pose)->{}, // Pose2d consumer, used to reset odometry at the beginning of auto
        Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
        new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
        new PIDConstants(0.8, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
        Swerve.getInstance()::setModuleStates, // Module states consumer used to output to the drive subsystem
        new HashMap<>(),
        false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
        Swerve.getInstance() // The drive subsystem. Used to properly set the requirements of path following commands
    );

    public static Command getTheThing(PathPoint notSaket, Rotation2d startHeading) {
        
        PathPoint startPoint = new PathPoint(Swerve.getInstance().getPose().getTranslation(), startHeading, Swerve.getInstance().getPose().getRotation());
        PathPoint endPoint = notSaket;

        SmartDashboard.putString("Auto Grab/Start Point", startPoint.name);
        SmartDashboard.putString("Auto Grab/End Point", endPoint.name);


        PathPlannerTrajectory traj1 = PathPlanner.generatePath(
            new PathConstraints(4, 3), 
            startPoint, // position, heading, holoroot
            endPoint
        );

        Command fullAuto = autoBuilder.fullAuto(traj1);
        return fullAuto;
        
    }

    public static Command getTheThing(PathPoint endPoint) {
        return getTheThing(endPoint, Swerve.getInstance().getPose().getRotation());
    }
}