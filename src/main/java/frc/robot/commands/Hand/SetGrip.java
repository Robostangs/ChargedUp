package frc.robot.commands.Hand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hand;

public class SetGrip extends CommandBase{

    private Hand mHand = Hand.getInstance();
    
    public SetGrip() {
        addRequirements(mHand);
        System.out.println("hi");
    }

    @Override
    public void execute() {
        mHand.setHolding(!mHand.getHolding());
    }
}
