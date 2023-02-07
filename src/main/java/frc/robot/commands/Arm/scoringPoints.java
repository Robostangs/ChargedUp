// package frc.robot.commands.Arm;

// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.commands.Hand.HandManager;
// import frc.robot.subsystems.Arm;
// import frc.robot.subsystems.Hand;
// import frc.robot.subsystems.Arm.ArmState;
// import frc.robot.subsystems.Hand.HandState;

// public class scoringPoints extends SequentialCommandGroup {
//     private static Arm mArm = Arm.getInstance();
//     private static Hand mHand = Hand.getInstance();

//     public scoringPoints(ArmState armState, HandState handState) {
//         addRequirements(mArm, mHand);
//         setName("Scoring Points");
//         addCommands(
//             new ArmManager(armState),
//             new HandManager(handState)
//         );
//     }
// }
