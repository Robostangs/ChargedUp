package frc.robot.commands.Hand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hand;

public class ToggleGrip extends CommandBase{

    private Hand mHand = Hand.getInstance();
    
    public ToggleGrip() {
        addRequirements(mHand);
        System.out.println("hi");
    }

    @Override
    public void execute() {
        mHand.setGripping(!mHand.getGripping());
    }
}
