package frc.robot.commands.RollyArm.Wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hand.Wrist;

public class wristDefault extends CommandBase {
    private final Wrist mWrist = Wrist.getInstance();

    /**
     * Default code to run the Wrist Feed Fwd
     */
    public wristDefault() {
        this.addRequirements(mWrist);
        this.setName("Default Command");
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        mWrist.wristFeedFwd();
    }
    
}
