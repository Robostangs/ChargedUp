package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.HashMap;
import java.util.function.Supplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class translatePp {
    /**  Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems. */
    static SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        Swerve.getInstance()::getPose, // Pose2d supplier
        (pose)->{}, // Pose2d consumer, used to reset odometry at the beginning of auto
        Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
        new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
        new PIDConstants(3.3, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
        Swerve.getInstance()::setModuleStates, // Module states consumer used to output to the drive subsystem
        new HashMap<>(),
        true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
        Swerve.getInstance() // The drive subsystem. Used to properly set the requirements of path following commands
    );

    public static Command getRelativeTranslateCommand(Supplier<PathPoint> relativeTranslateFieldCoordsSupplier) {
        return autoBuilder.followPath(()->getTrajectory(relativeTranslateFieldCoordsSupplier.get().withNewTranslation(relativeTranslateFieldCoordsSupplier.get().position.plus(Swerve.getInstance().getPose().getTranslation()))));
    }
    public static Command getAbsoluteTranslateCommand(Supplier<PathPoint> endPointSupplier) {
        return autoBuilder.followPath(()->PathPlannerTrajectory.transformTrajectoryForAlliance(getTrajectory(endPointSupplier.get()),DriverStation.getAlliance()));
    }

    public static Command getAbsoluteTranslateCommand(Supplier<PathPoint> endPointSupplier, Supplier<Rotation2d> startHeadingSupplier) {
        return autoBuilder.followPath(()-> PathPlannerTrajectory.transformTrajectoryForAlliance(getTrajectory(endPointSupplier.get(), startHeadingSupplier.get()), DriverStation.getAlliance()));
    }

    public static Command getCompleteTranslateCommand(Supplier<PathPoint> startPoint, Supplier<PathPoint> endPoint) {
        return autoBuilder.followPath(()-> PathPlannerTrajectory.transformTrajectoryForAlliance(getTrajectory(startPoint.get(), endPoint.get()), DriverStation.getAlliance()));
    }

    public static Command getMicroManageTranslateCommand(Supplier<PathPoint> startPointSupplier, Supplier<PathPoint> endPointSupplier, Supplier<PathPoint> midPointSupplier,  Supplier<Rotation2d> startHeadingSupplier) {
        return autoBuilder.followPath(()-> PathPlannerTrajectory.transformTrajectoryForAlliance(getTrajectory(startPointSupplier.get(), midPointSupplier.get(), endPointSupplier.get(), startHeadingSupplier.get()),DriverStation.getAlliance()));
    }
    
    public static PathPlannerTrajectory getTrajectory(PathPoint endPoint){
        return getTrajectory(endPoint, Swerve.getInstance().getPose().getRotation());
    }

    public static PathPlannerTrajectory getTrajectory(PathPoint endPoint, Rotation2d startHeading){
        PathPoint startPoint = new PathPoint(Swerve.getInstance().getPose().getTranslation(), startHeading, Swerve.getInstance().getPose().getRotation());

        SmartDashboard.putString("TranslatePp/Start Point", startPoint.name);
        SmartDashboard.putString("TranslatePp/End Point", endPoint.name);


        return PathPlanner.generatePath(
            new PathConstraints(4, 3), 
            startPoint, // position, heading, holoroot
            endPoint
        );
    }

    public static PathPlannerTrajectory getTrajectory(PathPoint startPoint, PathPoint endPoint) {
        SmartDashboard.putString("TranslatePp/Start Point", startPoint.name);
        SmartDashboard.putString("TranslatePp/End Point", endPoint.name);


        return PathPlanner.generatePath(
            new PathConstraints(4, 3), 
            startPoint, // position, heading, holoroot
            endPoint
        );
    }

    public static PathPlannerTrajectory getTrajectory(PathPoint startPoint, PathPoint midPoint, PathPoint endPoint, Rotation2d startHeading) {
        // PathPoint startPoint = new PathPoint(Swerve.getInstance().getPose().getTranslation(), startHeading, Swerve.getInstance().getPose().getRotation());

        SmartDashboard.putString("TranslatePp/Start Point", midPoint.name);
        SmartDashboard.putString("TranslatePp/End Point", endPoint.name);


        return PathPlanner.generatePath(
            new PathConstraints(4, 3), 
            startPoint, // position, heading, holoroot
            midPoint,
            endPoint
        );
    }
}