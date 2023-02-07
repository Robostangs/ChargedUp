// package frc.robot.commands.Arm;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants;
// import frc.robot.subsystems.Arm;
// import frc.robot.subsystems.Arm.ArmState;
// public class ArmManager extends CommandBase{

//     private static Arm mArm = Arm.getInstance();
//     private static ArmState currentState = ArmState.kStowedPosition;
//     private static ArmState mDesiredState;
    
//     public ArmManager(ArmState desiredState) {
//         addRequirements(mArm);
//         mDesiredState = desiredState;
//         setName("Arm Manager:" + mDesiredState);
//     }

//     @Override
//     public void initialize() {
//         mArm.setBrakeMode(false);
//     }

//     @Override
//     public void execute() {
//         if(mDesiredState != currentState) {
//             switch(mDesiredState) {
//                 case kStowedPosition:
//                     mArm.setArmPosition(Constants.Arm.Positions.stowPosition);
//                     break;
//                 case kIntakePosition:
//                     mArm.setArmPosition(Constants.Arm.Positions.intakePosition);
//                     break;
//                 case kLoadingZonePosition:
//                     mArm.setArmPosition(Constants.Arm.Positions.loadingStationPosition);
//                     break;
//                 case klowPosition:
//                     mArm.setArmPosition(Constants.Arm.Positions.lowPosition);
//                     break;
//                 case kmediumPosition:
//                     mArm.setArmPosition(Constants.Arm.Positions.mediumPosition);
//                     break;
//                 case khighPosition:
//                     mArm.setArmPosition(Constants.Arm.Positions.highPosition);
//                     break;
//             }
//         }
//     }

//     @Override
//     public void end(boolean interrupted) {
//         mArm.setArmPosition(Constants.Arm.Positions.stowPosition);
//         mArm.setBrakeMode(true);
//         currentState = mDesiredState;
//     }


// }
