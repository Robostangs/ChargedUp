package frc.robot.commands.Hand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hand;

public class SetHand extends CommandBase{

    private Hand mHand = Hand.getInstance();
    
    public SetHand() {
        addRequirements(mHand);
        System.out.println("hi");
    }

    @Override
    public void execute() {
        mHand.setSolenoid(true); 
    }

    @Override
    public void end(boolean interrupted) {
        mHand.setSolenoid(false);
    }
}
