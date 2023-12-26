package frc.robot.autos;

import java.io.IOException;
import java.nio.file.Path;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.Arm.ProfiledChangeSetPoint;
import frc.robot.commands.Hand.SetGrip;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Hand;
import frc.robot.subsystems.Swerve;

public class doubleAutoFromPath extends SequentialCommandGroup {
    public doubleAutoFromPath() {
        Swerve mSwerve = Swerve.getInstance();
        Arm mArm = Arm.getInstance();
        Hand mHand = Hand.getInstance();
        addRequirements(mSwerve, mArm, mHand);
        setName("autooooooooo");
        String path = "paths/" + Robot.chooser.getSelected();
        DataLogManager.log(path);

        // An example trajectory to follow. All units in meters.

        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath()
                    .resolve("paths/BlueBalanceLeftFirstPath.wpilib.json");
            Trajectory firstTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);

            trajectoryPath = Filesystem.getDeployDirectory().toPath()
                    .resolve("paths/BlueBalanceLeftSecondPath.wpilib.json");
        //     Trajectory secondTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            var thetaController = new ProfiledPIDController(
                    Constants.AutoConstants.kPThetaController, 0, 0,
                    Constants.AutoConstants.kThetaControllerConstraints);
            thetaController.enableContinuousInput(-Math.PI, Math.PI);

            // if(SmartDashboard.getBoolean("isRed", false)) {
            // mSwerve.setGyro(0);
            mSwerve.updateOdometryManual(firstTrajectory.getInitialPose().getX(),
                    firstTrajectory.getInitialPose().getY(),
                    firstTrajectory.getInitialPose().getRotation().getDegrees());
            // } else {
            // mSwerve.setGyro(180);
            // mSwerve.updateOdometryManual(exampleTrajectory.getInitialPose().getX(),
            // exampleTrajectory.getInitialPose().getY(), 0);
            // }
            SwerveControllerCommand firstController = new SwerveControllerCommand(
                    firstTrajectory,
                    mSwerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                    new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                    thetaController,
                    mSwerve::setModuleStates,
                    mSwerve);

        //     SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        //             secondTrajectory,
        //             mSwerve::getPose,
        //             Constants.Swerve.swerveKinematics,
        //             new PIDController(Constants.AutoConstants.kPXController, 0, 0),
        //             new PIDController(Constants.AutoConstants.kPYController, 0, 0),
        //             thetaController,
        //             mSwerve::setModuleStates,
        //             mSwerve);

            addCommands(
                ProfiledChangeSetPoint.createWithTimeout(()->Constants.Arm.SetPoint.coneHighPosition),
                    new WaitCommand(0.2),
                    new SetGrip().withTimeout(0.7),
                    new ParallelDeadlineGroup(
                        ProfiledChangeSetPoint.createWithTimeout(()->Constants.Arm.SetPoint.stowPosition),
                        new SetGrip()),
                    firstController,
                    new InstantCommand(
                            () -> mSwerve.drive(new Translation2d(0, 0), 0, false, false)),
                    new ParallelDeadlineGroup(
                        ProfiledChangeSetPoint.createWithTimeout(()->Constants.Arm.SetPoint.generalIntakePosition), // .andThen(new
                                                                                    // WaitCommand(0.2)),
                            new SetGrip()),
                            ProfiledChangeSetPoint.createWithTimeout(()->Constants.Arm.SetPoint.stowPosition));
                //     swerveControllerCommand,
                    // new balance());
        } catch (IOException ex) {

            DriverStation.reportError("Unable to Open Trajectory" + Filesystem.getDeployDirectory(),
                    ex.getStackTrace());
        }

    }
}
