// package frc.robot.commands.Drivetrain;

// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.Vision;
// import frc.robot.subsystems.Swerve;

// public class StraightenManager extends SequentialCommandGroup {
//     Swerve mDrivetrain = Swerve.getInstance();
//     Vision mVision = Vision.getInstance();

//     public StraightenManager(double moveSpeed, double angleSpeed, Vision.LimelightState state) {
//         addRequirements(mDrivetrain);
//         setName("Straighten Against Wall");

//         addCommands(
//             new Flatten(angleSpeed, state),
//             new GetToPosition(moveSpeed, state)
//         );
//     }


// }
