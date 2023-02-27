package frc.robot.commands.Hand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Hand;

public class ToggleHolding extends InstantCommand{

    private Hand mHand = Hand.getInstance();
    
    public ToggleHolding() {
        addRequirements(mHand);
    }

    @Override
    public void initialize() {
        System.out.println("hi");

        mHand.setHolding(!mHand.getHolding());
    }
}
