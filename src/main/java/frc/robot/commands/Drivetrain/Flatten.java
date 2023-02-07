// package frc.robot.commands.Drivetrain;

// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Vision;
// import frc.robot.subsystems.Swerve;

// public class Flatten extends CommandBase {
//     Swerve mDrivetrain = Swerve.getInstance();
//     Vision mVision = Vision.getInstance();
//     double mSpeed;
//     Vision.LimelightState mState;

//     public Flatten(double speed, Vision.LimelightState state) {
//         addRequirements(mDrivetrain);
//         setName("Straighten Against Wall");
//         mSpeed = speed;
//         mState = state;
//     }

//     @Override
//     public void execute() {
//         if(mDrivetrain.getGyroAngle() < 90 && mDrivetrain.getGyroAngle() > 0) {
//             mDrivetrain.drive(new Translation2d(0, 0), mSpeed, false, false);
//         } else if(mDrivetrain.getGyroAngle() > -90 && mDrivetrain.getGyroAngle() < 0) {
//             mDrivetrain.drive(new Translation2d(0, 0), -mSpeed, false, false);
//         }
//     }

//     @Override
//     public void end(boolean interrupted) {
//         mDrivetrain.drive(new Translation2d(0, 0), 0, false, false);
//     }

// }
