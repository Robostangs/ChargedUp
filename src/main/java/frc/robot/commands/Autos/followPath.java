package frc.robot.commands.Autos;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
import frc.robot.subsystems.Drivetrain;

public class followPath extends CommandBase {

    Trajectory mTrajectory = new Trajectory();
    Drivetrain mDrivetrain = Drivetrain.getInstance();
    String mPath = "";
    Double mSpeed;
    
    public followPath(String pathName, double speed) {
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
                Constants.Drivetrain.maxLinearVelocity,
                Constants.Drivetrain.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(Constants.Drivetrain.mKinematics);


        PIDController xController = new PIDController(Constants.Drivetrain.linearP, Constants.Drivetrain.linearI, Constants.Drivetrain.linearD);
        PIDController yController = new PIDController(Constants.Drivetrain.linearP, Constants.Drivetrain.linearI, Constants.Drivetrain.linearD);

        ProfiledPIDController thetaController = new ProfiledPIDController(
               Constants.Drivetrain.angularP,
               Constants.Drivetrain.angularI,
               Constants.Drivetrain.angularD,
               new TrapezoidProfile.Constraints(
                Constants.Drivetrain.maxAngularVelocity,
                Constants.Drivetrain.kMaxAngularAccelerationRadiansPerSecondSquared));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                mTrajectory,
                mDrivetrain.getPose(),
                Constants.Drivetrain.mKinematics,
                xController,
                yController,
                thetaController,
                mDrivetrain::setModuleStates,
                mDrivetrain);

        // 5. Add some init and wrap-up, and return everything
        new SequentialCommandGroup(
                new InstantCommand(() -> mDrivetrain.resetOdometry(mTrajectory.getInitialPose())),
                swerveControllerCommand,
                new InstantCommand(() -> mDrivetrain.drive(new ChassisSpeeds(0,0,0))));
    }

    @Override
    public void end(boolean interrupted) {
        mDrivetrain.drive(new ChassisSpeeds(0, 0, 0));
    }

}
