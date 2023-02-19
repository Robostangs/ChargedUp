package frc.robot.commands.Hand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hand;
import frc.robot.subsystems.Hand.HandState;
public class SetHand extends CommandBase{

    private static Hand mHand = Hand.getInstance();
    
    public SetHand() {
        addRequirements(mHand);
    }

    @Override
    public void execute() {
        mHand.setSolenoid(true); 
    }

    @Override
    public void end(boolean interrupted) {
        mHand.setSolenoid(false);;
    }


}
