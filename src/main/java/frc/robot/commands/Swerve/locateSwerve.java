package frc.robot.commands.Swerve;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.Utils.Vector2D;
import frc.robot.autos.translatePp;
import frc.robot.subsystems.Swerve;

public class locateSwerve extends SubsystemBase {
    private Swerve mSwerve;
    private SwerveModule Mod1, Mod2, Mod3, Mod4; // Front Left, Front Right, Back Left, Back Right
    private Pigeon2 gyro;
    private Object Mod1_Distance, Mod2_Distance, Mod3_Distance, Mod4_Distance;
    // private final double homeX;
    // private final double homeY;
    // private final double homeZ;
    private Pose2d home;
    private Pose2d chargePointPose2d = new Pose2d(2, 2, new Rotation2d(0));
    private Translation2d chargePointLeft = new Translation2d(2, 2);

    PIDController xController = new PIDController(Constants.AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(Constants.AutoConstants.kPYController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
    
    
    TrajectoryConfig trajectoryConfig = 
    new TrajectoryConfig(
        Constants.AutoConstants.kMaxSpeedMetersPerSecond,
        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared
    ).setKinematics(Constants.Swerve.swerveKinematics);

    SwerveControllerCommand swerveControllerCommand;

    private final SwerveAutoBuilder autoBuilder;

    public locateSwerve(Pigeon2 pigeon, Swerve swerve) {
        gyro = pigeon;
        mSwerve = swerve;
        Mod1 = mSwerve.mSwerveMods[1];
        Mod2 = mSwerve.mSwerveMods[0];
        Mod3 = mSwerve.mSwerveMods[2];
        Mod4 = mSwerve.mSwerveMods[3];
        home = mSwerve.getPose();

        autoBuilder = new SwerveAutoBuilder(
            () -> home,
            home -> {},
            Constants.Swerve.swerveKinematics,
            new PIDConstants(5.0, 0.0, 0.0),
            new PIDConstants(3.3, 0.0, 0.0),
            mSwerve::setModuleStates,
            new HashMap<>(),
            true,
            mSwerve
        );
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public double robotX() {
        return mSwerve.getPose().getX();
    }

    public double robotY() {
        return mSwerve.getPose().getY();
    }

    public Command returnHome() {
        // return goTo(home, 3.0, 3.0);

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            mSwerve.getPose(),
            List.of(chargePointLeft),
            home,
            trajectoryConfig
        );

        PathPlannerTrajectory pathPlannerTrajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(
            PathPlanner.generatePath(
                new PathConstraints(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared
                ), new PathPoint(
                    home.getTranslation(),
                    home.getRotation(), -1
                ), new PathPoint(
                    chargePointPose2d.getTranslation(),
                    chargePointPose2d.getRotation(), -1)
            ), DriverStation.getAlliance()
        );

        swerveControllerCommand = new SwerveControllerCommand(
            pathPlannerTrajectory,
            // trajectory,
            mSwerve::getPose,
            Constants.Swerve.swerveKinematics,
            xController, yController, thetaController,
            mSwerve::setModuleStates,
            mSwerve
        );

        return swerveControllerCommand;
    }

    public Command goTo(Pose2d setPoint, double maxAcceleration, double maxSpeed) {
        return autoBuilder.followPath(PathPlannerTrajectory.transformTrajectoryForAlliance(PathPlanner.generatePath(
                new PathConstraints(maxSpeed, maxAcceleration), // Path Constraints
                new PathPoint(mSwerve.getPose().getTranslation(), mSwerve.getPose().getRotation(),
                        mSwerve.getPose().getRotation()), // StartPoint
                new PathPoint(new Translation2d(setPoint.getX(), setPoint.getY()), setPoint.getRotation(),
                        setPoint.getRotation()) // EndPoint
        ), DriverStation.getAlliance()));
    }
}