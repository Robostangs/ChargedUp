package frc.robot.autos;

import java.io.IOException;
import java.nio.file.Path;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
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
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.Arm.SetArmPosition;
import frc.robot.commands.Hand.SetGrip;
import frc.robot.commands.Swerve.balance;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Hand;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Arm.ArmPosition;

public class doubleAutoFromPath extends SequentialCommandGroup {
    public doubleAutoFromPath() {
        Swerve s_Swerve = Swerve.getInstance();
        Arm s_Arm = Arm.getInstance();
        Hand s_Hand = Hand.getInstance();
        addRequirements(s_Swerve, s_Arm, s_Hand);
        setName("autooooooooo");
        String path = "paths/" + Robot.chooser.getSelected();
        DataLogManager.log(path);

        TrajectoryConfig config = new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(Constants.Swerve.swerveKinematics);

        // An example trajectory to follow. All units in meters.

        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath()
                    .resolve("paths/BlueBalanceLeftFirstPath.wpilib.json");
            Trajectory firstTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);

            trajectoryPath = Filesystem.getDeployDirectory().toPath()
                    .resolve("paths/BlueBalanceLeftSecondPath.wpilib.json");
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
            SwerveControllerCommand firstController = new SwerveControllerCommand(
                    firstTrajectory,
                    s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                    new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                    thetaController,
                    s_Swerve::setModuleStates,
                    s_Swerve);

            SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                    secondTrajectory,
                    s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                    new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                    thetaController,
                    s_Swerve::setModuleStates,
                    s_Swerve);

            addCommands(
                    new SetArmPosition(ArmPosition.kHighPosition).withTimeout(5),
                    new WaitCommand(0.2),
                    new SetGrip().withTimeout(0.7),

                    new SetArmPosition(ArmPosition.kStowPosition).withTimeout(3),
                    firstController,
                    new InstantCommand(
                            () -> s_Swerve.drive(new Translation2d(0, 0), 0, false, false)),
                    new ParallelDeadlineGroup(
                    new SetArmPosition(ArmPosition.kIntakePositionGeneral), // .andThen(new
                                                                            // WaitCommand(0.2)),
                    new SetGrip()
                    ),
                    new SetArmPosition(ArmPosition.kStowPosition).withTimeout(3),
                    swerveControllerCommand,
                    new balance());
        } catch (IOException ex) {

            DriverStation.reportError("Unable to Open Trajectory" + Filesystem.getDeployDirectory(),
                    ex.getStackTrace());
        }

    }
}
