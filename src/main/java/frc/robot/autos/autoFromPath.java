package frc.robot.autos;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Utils;
import frc.robot.commands.Arm.SetArmPosition;
import frc.robot.commands.Autos.balance;
import frc.robot.commands.Hand.SetGrip;
import frc.robot.commands.Hand.ToggleGrip;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Hand;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Arm.ArmPosition;

public class autoFromPath extends SequentialCommandGroup {
    public autoFromPath() {
        Swerve s_Swerve = Swerve.getInstance();
        Arm s_Arm = Arm.getInstance();
        Hand s_Hand = Hand.getInstance();
        addRequirements(s_Swerve, s_Arm, s_Hand
        );
        TrajectoryConfig config = new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

        // An example trajectory to follow. All units in meters.


        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("paths/BalanceSpotFarNearLink.wpilib.json");
            Trajectory exampleTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            var thetaController = new ProfiledPIDController(
                    Constants.AutoConstants.kPThetaController, 0, 0,
                    Constants.AutoConstants.kThetaControllerConstraints);
            thetaController.enableContinuousInput(-Math.PI, Math.PI);

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
                    new SetArmPosition(ArmPosition.kHighPosition, Hand.getInstance().getHolding()),
                    new WaitCommand(0.75),
                    new SetGrip().withTimeout(0.7),
                    new SetArmPosition(ArmPosition.kStowPosition, Hand.getInstance().getHolding()),
                    new WaitCommand(0.5),
                    new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialPose())),
                    swerveControllerCommand,
                    new balance()
            );
        } catch (IOException ex) {

            DriverStation.reportError("Unable to Open Trajectory" + Filesystem.getDeployDirectory(), ex.getStackTrace());
        }

    }
}
