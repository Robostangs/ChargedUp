// package frc.robot.commands.Hand;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.HandNormal;

// public class SetGrip extends CommandBase{

//     private HandNormal mHand = HandNormal.getInstance();
    
//     /**
//      * Will Close the Claw, and then Open the claw in end() function
//      */
//     public SetGrip() {
//         addRequirements(mHand);
//     }

//     @Override
//     public void execute() {
//         mHand.setGripping(true);
//     }

//     @Override
//     public void end(boolean interrupted) {
//         mHand.setGripping(false);
//         super.end(interrupted);
//     }
// }
