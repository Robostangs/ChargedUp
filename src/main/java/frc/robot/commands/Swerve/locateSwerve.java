package frc.robot.commands.Swerve;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class locateSwerve {
    private final Swerve mSwerve = Swerve.getInstance();
    private Pose2d home;   /** Check {@link Constants.Swerve.homePoints} */
    // private Pose2d setPoint;
    // public ArrayList<Pose2d> homePoints = new ArrayList<Pose2d>();
    // public enum homePoints {
    //     kBlueLeft(new Pose2d(2.2, 4.65, new Rotation2d(180)))
    // }
    // public value;
    
    private final Pose2d chargePointPose2d = new Pose2d(5.69, 4.65, new Rotation2d(180));

    PIDController xController = new PIDController(Constants.AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(Constants.AutoConstants.kPYController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
    
    
    // TrajectoryConfig trajectoryConfig = 
    //                 new TrajectoryConfig(
    //                     Constants.AutoConstants.kMaxSpeedMetersPerSecond,
    //                     Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared
    //                 ).setKinematics(Constants.Swerve.swerveKinematics);

    // SwerveControllerCommand swerveControllerCommand;

    // private final SwerveAutoBuilder autoBuilder;
    /**
     * 
     * @param posePoint see {@link Constants.Swerve.homePoints} for home points
     * @param isHomePoint boolean if the input is the home point of midpoint
     */
    // public locateSwerve(Pose2d posePoint, boolean isHomePoint) {
        // if (isHomePoint) {
    //         home = posePoint;
    //     } else {
    //         setPoint = posePoint;
    //     }
    //     thetaController.enableContinuousInput(-Math.PI, Math.PI);
    // }

    public locateSwerve(Pose2d homePoint) {
        home = homePoint;
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public Command returnHome() {
        return new SwerveControllerCommand(
            PathPlannerTrajectory.transformTrajectoryForAlliance(
                PathPlanner.generatePath(
                    new PathConstraints(
                        Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared
                    ), new PathPoint(
                        home.getTranslation(),
                        new Rotation2d(0),
                        home.getRotation()
                    ), new PathPoint(
                        chargePointPose2d.getTranslation(),
                        new Rotation2d(0),
                        chargePointPose2d.getRotation()
            )), DriverStation.getAlliance()),
            mSwerve::getPose,
            Constants.Swerve.swerveKinematics,
            xController, yController, thetaController,
            mSwerve::setModuleStates,
            mSwerve
        );
    }

    public double robotX() {
        return mSwerve.getPose().getX();
    }

    public double robotY() {
        return mSwerve.getPose().getY();
    }

    // public Command returnHomeOld() {
        // return goTo(home, 3.0, 3.0);

        // Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        //     mSwerve.getPose(),
        //     List.of(chargePointPose2d.getTranslation()),
        //     home,
        //     trajectoryConfig
        // );

        // PathPlannerTrajectory pathPlannerTrajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(
        //     PathPlanner.generatePath(
        //         new PathConstraints(
        //             Constants.AutoConstants.kMaxSpeedMetersPerSecond,
        //             Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared
        //         ), new PathPoint(
        //             home.getTranslation(),
        //             home.getRotation(),
        //             home.getRotation()
        //         ), new PathPoint(
        //             chargePointPose2d.getTranslation(),
        //             chargePointPose2d.getRotation(),
        //             home.getRotation()
        //     )), DriverStation.getAlliance()
        // );
    // }

    // public Command goTo(Pose2d setPoint, double maxAcceleration, double maxSpeed) {
    //     return autoBuilder.followPath(PathPlannerTrajectory.transformTrajectoryForAlliance(PathPlanner.generatePath(
    //             new PathConstraints(maxSpeed, maxAcceleration), // Path Constraints
    //             new PathPoint(mSwerve.getPose().getTranslation(), mSwerve.getPose().getRotation(),
    //                     mSwerve.getPose().getRotation()), // StartPoint
    //             new PathPoint(new Translation2d(setPoint.getX(), setPoint.getY()), setPoint.getRotation(),
    //                     setPoint.getRotation()) // EndPoint
    //     ), DriverStation.getAlliance()));
    // }
}