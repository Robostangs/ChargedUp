package frc.robot.commands.Hand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Hand;

public class SetHolding extends InstantCommand{

    private Hand mHand = Hand.getInstance();
    
    public SetHolding() {
        addRequirements(mHand);
    }

    @Override
    public void initialize() {
        mHand.setHolding(!mHand.getHolding());
    }
}
