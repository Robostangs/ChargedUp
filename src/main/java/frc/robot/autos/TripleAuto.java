package frc.robot.autos;

import java.io.IOException;
import java.nio.file.Path;

import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.LoggyThings.LoggyPrintCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.Arm.ProfiledChangeSetPoint;
import frc.robot.commands.Hand.SetGrip;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Hand;
import frc.robot.subsystems.Swerve;

public class TripleAuto extends SequentialCommandGroup {
        public TripleAuto() {
                Swerve s_Swerve = Swerve.getInstance();
                Arm s_Arm = Arm.getInstance();
                Hand s_Hand = Hand.getInstance();
                addRequirements(s_Swerve, s_Arm, s_Hand);
                setName("Triple Auto");
                String path = "paths/" + Robot.chooser.getSelected();
                DataLogManager.log(path);

                TrajectoryConfig config = new TrajectoryConfig(
                                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                                .setKinematics(Constants.Swerve.swerveKinematics);

                // An example trajectory to follow. All units in meters.

                try {
                        Path trajectoryPath = Filesystem.getDeployDirectory().toPath()
                                        .resolve("paths/Expiremental/firstPath.wpilib.json");
                        Trajectory firstTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);

                        trajectoryPath = Filesystem.getDeployDirectory().toPath()
                                        .resolve("paths/Expiremental/secondPath.wpilib.json");
                        Trajectory secondTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);

                        var thetaController = new ProfiledPIDController(
                                        Constants.AutoConstants.kPThetaController, 0, 0,
                                        Constants.AutoConstants.kThetaControllerConstraints);
                        thetaController.enableContinuousInput(-Math.PI, Math.PI);

                        // if(SmartDashboard.getBoolean("isRed", false)) {
                        // s_Swerve.setGyro(0);
                        s_Swerve.updateOdometryManual(firstTrajectory.getInitialPose().getX(),
                                        firstTrajectory.getInitialPose().getY(),
                                        firstTrajectory.getInitialPose().getRotation().getDegrees());
                        // } else {
                        // s_Swerve.setGyro(180);
                        // s_Swerve.updateOdometryManual(exampleTrajectory.getInitialPose().getX(),
                        // exampleTrajectory.getInitialPose().getY(), 0);
                        // }

                        /*
                         * TODO:
                         * Add more to P of rotation so it rotates more after starting path
                         * Make the starting auto place more accurate
                         * Allow the auto grab command to properly end
                         * TEST
                         */

                        SwerveControllerCommand firstController = new SwerveControllerCommand(
                                        firstTrajectory,
                                        s_Swerve::getPose,
                                        Constants.Swerve.swerveKinematics,
                                        new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                        new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                        thetaController,
                                        s_Swerve::setModuleStates,
                                        s_Swerve);

                        SwerveControllerCommand secondController = new SwerveControllerCommand(
                                        secondTrajectory,
                                        s_Swerve::getPose,
                                        Constants.Swerve.swerveKinematics,
                                        new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                        new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                        thetaController,
                                        s_Swerve::setModuleStates,
                                        s_Swerve);
                        new charlieAutoGrab();
                        addCommands(
                                        // ProfiledChangeSetPoint.createWithTimeout(()->Constants.Arm.SetPoint.coneHighPosition),
                                        // new WaitCommand(0.2),
                                        // new SetGrip().withTimeout(0.7),
                                        // new ParallelDeadlineGroup(
                                        // ProfiledChangeSetPoint.createWithTimeout(()->Constants.Arm.SetPoint.stowPosition),
                                        // new SetGrip()),
                                        // firstController,
                                        // new charlieAutoDriveToCube(),
                                        // ProfiledChangeSetPoint.createWithTimeout(()->Constants.Arm.SetPoint.stowPosition),
                                        // secondController

                                        ProfiledChangeSetPoint.createWithTimeout(
                                                        () -> Constants.Arm.SetPoint.coneHighPositionBad),
                                        new WaitCommand(0.5), new InstantCommand(() -> s_Arm.resetLash()),
                                        new SetGrip().withTimeout(0.7),
                                        new ParallelDeadlineGroup(
                                                        new ParallelDeadlineGroup(
                                                                        ProfiledChangeSetPoint.createWithTimeout(
                                                                                        () -> Constants.Arm.SetPoint.stowPosition),
                                                                        new SetGrip()).withTimeout(2.5)),
                                        translatePpNotRelative.getTheThing(new PathPoint(new Translation2d(5.59, 4.361),
                                                        Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
                                                        Rotation2d.fromDegrees(Math.abs(Swerve.getInstance().getPose()
                                                                        .getRotation().getDegrees() - 180))),
                                        // new InstantCommand(() -> s_Swerve.resetOdometry(new )),

                                        charlieAutoGrab.getCommand().withTimeout(2.5)
                                                        .andThen(new LoggyPrintCommand("exited")),
                                        // new LoggyPrintCommand("exited2"),

                                        new ParallelDeadlineGroup(new SequentialCommandGroup(
                                                        ProfiledChangeSetPoint.createWithTimeout(
                                                                        () -> Constants.Arm.SetPoint.cubeHighPosition),
                                                        new WaitCommand(0.5),
                                                        new InstantCommand(() -> s_Arm.resetLash()),
                                                        new SetGrip().withTimeout(0.7),
                                                        new ParallelDeadlineGroup(
                                                                        ProfiledChangeSetPoint.createWithTimeout(
                                                                                        () -> Constants.Arm.SetPoint.stowPosition),
                                                                        new SetGrip()).withTimeout(2.5)),
                                                        translatePpNotRelative
                                                                        .getTheThing(new PathPoint(
                                                                                        new Translation2d(1.836, 4.473),
                                                                                        Rotation2d.fromDegrees(0),
                                                                                        Rotation2d.fromDegrees(180)))
                                                                        .andThen(new LoggyPrintCommand("Continued")))

                        );
                } catch (IOException ex) {

                        DriverStation.reportError("Unable to Open Trajectory" + Filesystem.getDeployDirectory(),
                                        ex.getStackTrace());
                }

        }
}
