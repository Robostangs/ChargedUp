package frc.robot.commands.Hand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hand;

public class SetHolding extends CommandBase{

    private Hand mHand = Hand.getInstance();
    
    public SetHolding() {
        addRequirements(mHand);
        System.out.println("hi");
    }

    @Override
    public void execute() {
        mHand.setHandHolding(true);
    }

    @Override
    public void end(boolean interrupted) {
        mHand.setHandHolding(false);
        super.end(interrupted);
    }
}package frc.robot.commands.Hand;

public class SetHolding {
    
}
