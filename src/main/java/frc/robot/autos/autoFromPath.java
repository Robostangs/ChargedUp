package frc.robot.autos;

import java.io.IOException;
import java.nio.file.Path;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
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
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.Arm.ProfiledChangeSetPoint;
import frc.robot.commands.Hand.SetGrip;
import frc.robot.commands.Swerve.balance;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Hand;
import frc.robot.subsystems.Swerve;

public class autoFromPath extends SequentialCommandGroup {
        public autoFromPath() {
                Swerve s_Swerve = Swerve.getInstance();
                Arm s_Arm = Arm.getInstance();
                Hand s_Hand = Hand.getInstance();
                addRequirements(s_Swerve, s_Arm, s_Hand);
                String path = "paths/" + Robot.chooser.getSelected();
                DataLogManager.log(path);

                if(path.equals("paths/tripleAuto")) {
                        addCommands(new TripleAuto());
                } else {

                TrajectoryConfig config = new TrajectoryConfig(
                                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                                .setKinematics(Constants.Swerve.swerveKinematics);

                // An example trajectory to follow. All units in meters.

                try {
                        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(path);
                        Trajectory exampleTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
                        var thetaController = new ProfiledPIDController(
                                        Constants.AutoConstants.kPThetaController, 0, 0,
                                        Constants.AutoConstants.kThetaControllerConstraints);
                        thetaController.enableContinuousInput(-Math.PI, Math.PI);

                        // if(SmartDashboard.getBoolean("isRed", false)) {
                        // s_Swerve.setGyro(0);
                        s_Swerve.updateOdometryManual(exampleTrajectory.getInitialPose().getX(),
                                        exampleTrajectory.getInitialPose().getY(),
                                        exampleTrajectory.getInitialPose().getRotation().getDegrees());
                        // } else {
                        // s_Swerve.setGyro(180);
                        // s_Swerve.updateOdometryManual(exampleTrajectory.getInitialPose().getX(),
                        // exampleTrajectory.getInitialPose().getY(), 0);
                        // }
                        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                                        exampleTrajectory,
                                        s_Swerve::getPose,
                                        Constants.Swerve.swerveKinematics,
                                        new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                        new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                        thetaController,
                                        s_Swerve::setModuleStates,
                                        s_Swerve);

                        addCommands(
                                        ProfiledChangeSetPoint.createWithTimeout(()->Constants.Arm.SetPoint.coneHighPosition),
                                        new WaitCommand(0.5), new InstantCommand(() -> s_Arm.resetLash()),
                                        new SetGrip().withTimeout(0.7),
                                        new ParallelDeadlineGroup(
                                                        ProfiledChangeSetPoint.createWithTimeout(()->Constants.Arm.SetPoint.stowPosition),
                                                        new SetGrip()).withTimeout(2.5),
                                        // new InstantCommand(() -> s_Swerve.resetOdometry(new )),
                                        swerveControllerCommand,
                                        new balance());
                } catch (IOException ex) {

                        DriverStation.reportError("Unable to Open Trajectory" + Filesystem.getDeployDirectory(),
                                        ex.getStackTrace());
                }
        }
        }
}
