package frc.robot.commands.RollyArm.Wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hand.Wrist;

public class wristDefault extends CommandBase {
    private final Wrist mHand = Wrist.getInstance();

    public wristDefault() {
        this.addRequirements(mHand);
        this.setName("Default Command");
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        mHand.wristFeedFwd();
    }
    
}
