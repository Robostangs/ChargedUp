package frc.robot.commands.Hand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hand;

public class SetGrip extends CommandBase{

    private Hand mHand = Hand.getInstance();
    
    public SetGrip() {
        addRequirements(mHand);
    }

    @Override
    public void execute() {
        mHand.setGripping(true);
    }

    @Override
    public void end(boolean interrupted) {
        mHand.setGripping(false);
        super.end(interrupted);
    }
}
