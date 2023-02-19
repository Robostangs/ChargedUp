package frc.robot.commands.Autos;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class FollowPath extends CommandBase {

    Trajectory mTrajectory = new Trajectory();
    Swerve mDrivetrain = Swerve.getInstance();
    String mPath = "";
    Double mSpeed;
    
    public FollowPath(String pathName, double speed) {
        addRequirements(mDrivetrain);
        setName("Follow Path" + pathName);
        mPath = pathName;
        mSpeed = speed;
        
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(mPath);
            mTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
         } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + mPath, ex.getStackTrace());
         }
    }

    @Override
    public void execute() {
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(Constants.Swerve.swerveKinematics);


        PIDController xController = new PIDController(Constants.Swerve.driveKP, Constants.Swerve.driveKI, Constants.Swerve.driveKD);
        PIDController yController = new PIDController(Constants.Swerve.driveKP, Constants.Swerve.driveKI, Constants.Swerve.driveKD);

        ProfiledPIDController thetaController = new ProfiledPIDController(
               Constants.Swerve.angleKP,
               Constants.Swerve.angleKI,
               Constants.Swerve.angleKD,
               new TrapezoidProfile.Constraints(
                Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond,
                Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                mTrajectory,
                mDrivetrain::getPose,
                Constants.Swerve.swerveKinematics,
                xController,
                yController,
                thetaController,
                mDrivetrain::setModuleStates,
                mDrivetrain);

        // 5. Add some init and wrap-up, and return everything
        new SequentialCommandGroup(
                new InstantCommand(() -> mDrivetrain.resetOdometry(mTrajectory.getInitialPose())),
                swerveControllerCommand,
                new InstantCommand(() -> mDrivetrain.drive(new Translation2d(0, 0), 0, false, false)));
    }

    @Override
    public void end(boolean interrupted) {
        mDrivetrain.drive(new Translation2d(0, 0), 0, false, false);

    }

}
