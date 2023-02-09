package frc.robot.commands.Hand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hand;
import frc.robot.subsystems.Hand.HandState;
public class HandManager extends CommandBase{

    private static Hand mHand = Hand.getInstance();
    private static HandState currentState = HandState.CLOSED;
    private static HandState mDesiredState;
    
    public HandManager(HandState handState) {
        addRequirements(mHand);
        mDesiredState = handState;
        setName("Arm Manager:" + mDesiredState);
    }

    @Override
    public void execute() {
        if(mDesiredState != currentState) {
            switch(mDesiredState) {
                case OPEN:
                    mHand.setSolenoid(false);
                    break;
                case CLOSED:
                    mHand.setSolenoid(true);
                    break;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        currentState = mDesiredState;
    }


}
