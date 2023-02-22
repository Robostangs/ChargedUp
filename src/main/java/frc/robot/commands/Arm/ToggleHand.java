package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hand;

public class ToggleHand extends CommandBase{

    Hand mHand = Hand.getInstance();
    
    public ToggleHand() {
        addRequirements(mHand);
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
