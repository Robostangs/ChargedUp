// package frc.robot.commands.Swerve;

// import java.nio.file.Path;
// import java.util.ArrayList;

// import com.fasterxml.jackson.core.JsonFactoryBuilder;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.TrajectoryUtil;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.Filesystem;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants;
// import frc.robot.Utils;
// import frc.robot.subsystems.Swerve;

// public class GoToRelativePosition extends CommandBase {

//     private Swerve mSwerve = Swerve.getInstance();
//     private Utils.Vector3D mSetPoints;
//     private Trajectory mTrajectory;
//     private Double mSpeed;


//     public GoToRelativePosition(Utils.Vector3D setPoints, double speed) {
//         mSetPoints = setPoints;
//         mSpeed = speed;
//         addRequirements(mSwerve);
//     }

//     @Override
//     public void initialize() {
//         mSetPoints.x += mSwerve.swerveOdometry.getPoseMeters().getX();
//         mSetPoints.y += mSwerve.swerveOdometry.getPoseMeters().getY();
//         mSetPoints.z += mSwerve.getGyroAngle();

//         mTrajectory = new Trajectory().;
//     }

//     @Override
//     public void execute() {
//         TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
//             Constants.AutoConstants.kMaxSpeedMetersPerSecond,
//             Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
//                     .setKinematics(Constants.Swerve.swerveKinematics);


//     PIDController xController = new PIDController(Constants.Swerve.driveKP, Constants.Swerve.driveKI, Constants.Swerve.driveKD);
//     PIDController yController = new PIDController(Constants.Swerve.driveKP, Constants.Swerve.driveKI, Constants.Swerve.driveKD);

//     ProfiledPIDController thetaController = new ProfiledPIDController(
//            Constants.Swerve.angleKP,
//            Constants.Swerve.angleKI,
//            Constants.Swerve.angleKD,
//            new TrapezoidProfile.Constraints(
//             Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond,
//             Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared));
//     thetaController.enableContinuousInput(-Math.PI, Math.PI);

//     SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
//             mTrajectory,
//             mSwerve::getPose,
//             Constants.Swerve.swerveKinematics,
//             xController,
//             yController,
//             thetaController,
//             mSwerve::setModuleStates,
//             mSwerve);

//     // 5. Add some init and wrap-up, and return everything
//     new SequentialCommandGroup(
//             new InstantCommand(() -> mSwerve.resetOdometry(mTrajectory.getInitialPose())),
//             swerveControllerCommand,
//             new InstantCommand(() -> mSwerve.drive(new Translation2d(0, 0), 0, false, true)));
//     }
    

//     @Override
//     public void end(boolean interrupted) {
//         mSwerve.drive(new Translation2d(0, 0), 0, false, true);
//     }
// }
