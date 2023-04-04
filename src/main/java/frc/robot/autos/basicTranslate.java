package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.Utils;
import frc.robot.Utils.Vector3D;
import frc.robot.subsystems.Swerve;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class basicTranslate extends CommandBase {
    private Swerve s_Swerve = Swerve.getInstance();
    private Vector3D position;
    private TrajectoryConfig config;
    public basicTranslate(Swerve s_Swerve, Utils.Vector3D position){
        this.position = position;
        addRequirements(s_Swerve);
        config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared*0.5)
                .setKinematics(Constants.Swerve.swerveKinematics);
    }

    @Override
    public void initialize() {
        // An example trajectory to follow.  All units in meters.
        new Rotation2d();
        Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                // new Pose2d(s_Swerve.getPose().getX(), s_Swerve.getPose().getY(), Rotation2d.fromDegrees(s_Swerve.getGyroAngle())),
                new Pose2d(s_Swerve.getPose().getX(), s_Swerve.getPose().getY(), Rotation2d.fromDegrees(s_Swerve.getGyroAngle())),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(
                        // new Translation2d(s_Swerve.getPose().getX() + position, 0)
                        ),
                // End 3 meters straight ahead of where we started, facing forward
                // new Pose2d(position.x, position.y, Rotation2d.fromDegrees((position.y >= Constants.fieldLength/2) ? 0 : 180)),
                new Pose2d(s_Swerve.getPose().getX() - position.x, s_Swerve.getPose().getY() + position.y, Rotation2d.fromDegrees(s_Swerve.getGyroAngle())),
                config);


        var thetaController =
            new ProfiledPIDController(
                0, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                exampleTrajectory,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 1, 0),
                new PIDController(Constants.AutoConstants.kPYController, 1, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);


        new SequentialCommandGroup(
            // new InstantCommand(() -> new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialPose()))),
            swerveControllerCommand.deadlineWith(new RunCommand(() -> DataLogManager.log(s_Swerve.getPose().getX() + ", " + s_Swerve.getPose().getY())))
        ).schedule();
    }
}