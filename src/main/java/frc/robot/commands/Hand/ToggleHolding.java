// package frc.robot.commands.Hand;

// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.GenericHID.RumbleType;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import frc.robot.subsystems.HandNormal;

// public class ToggleHolding extends InstantCommand{

//     private HandNormal mHand = HandNormal.getInstance();
//     private XboxController mManipController = new XboxController(1);

//     /**
//      * Change the Cone Holding State, does not affect Claw State
//      */
//     public ToggleHolding() {
//         addRequirements(mHand);
//     }

//     @Override
//     public void initialize() {
//         mHand.setHolding(!mHand.getHolding());
        
//         if(mHand.getHolding()) {
//             mManipController.setRumble(RumbleType.kLeftRumble, 0.25);
//         } else {
//             mManipController.setRumble(RumbleType.kRightRumble, 0.25);
//         }
//     }
// }
