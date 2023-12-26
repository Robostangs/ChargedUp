package frc.robot.autos;

import java.io.IOException;
import java.nio.file.Path;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.Swerve.balance;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Hand;
import frc.robot.subsystems.Swerve;

public class justAuto extends SequentialCommandGroup {
        public justAuto() {
                Swerve mSwerve = Swerve.getInstance();
                Arm mArm = Arm.getInstance();
                Hand mHand = Hand.getInstance();
                addRequirements(mSwerve, mArm, mHand);
                String path = "paths/" + Robot.chooser.getSelected();
                DataLogManager.log(path);

                // An example trajectory to follow. All units in meters.

                try {
                        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(path);
                        Trajectory exampleTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
                        var thetaController = new ProfiledPIDController(
                                        Constants.AutoConstants.kPThetaController, 0, 0,
                                        Constants.AutoConstants.kThetaControllerConstraints);
                        thetaController.enableContinuousInput(-Math.PI, Math.PI);

                        // if(SmartDashboard.getBoolean("isRed", false)) {
                        // mSwerve.setGyro(0);
                        mSwerve.updateOdometryManual(exampleTrajectory.getInitialPose().getX(),
                                        exampleTrajectory.getInitialPose().getY(),
                                        exampleTrajectory.getInitialPose().getRotation().getDegrees());
                        // } else {
                        // mSwerve.setGyro(180);
                        // mSwerve.updateOdometryManual(exampleTrajectory.getInitialPose().getX(),
                        // exampleTrajectory.getInitialPose().getY(), 0);
                        // }
                        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                                        exampleTrajectory,
                                        mSwerve::getPose,
                                        Constants.Swerve.swerveKinematics,
                                        new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                        new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                        thetaController,
                                        mSwerve::setModuleStates,
                                        mSwerve);

                        addCommands(
                                        // new SetArmPosition(ArmPosition.kHighPosition).withTimeout(5),
                                        new WaitCommand(0.5), new InstantCommand(() -> mArm.resetLash()),
                                        // new SetGrip().withTimeout(0.7),
                                        // new ParallelDeadlineGroup(
                                        //                 new SetArmPosition(ArmPosition.kStowPosition),
                                        //                 new SetGrip()).withTimeout(2.5),
                                        // // new InstantCommand(() -> mSwerve.resetOdometry(new )),
                                        swerveControllerCommand,
                                        new balance());
                } catch (IOException ex) {

                        DriverStation.reportError("Unable to Open Trajectory" + Filesystem.getDeployDirectory(),
                                        ex.getStackTrace());
                }

        }
}
