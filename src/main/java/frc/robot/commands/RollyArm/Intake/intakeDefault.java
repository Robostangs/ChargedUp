package frc.robot.commands.RollyArm.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hand.Intake;

public class intakeDefault extends CommandBase {
    private final Intake mIntake = Intake.getInstance();

    public intakeDefault() {
        this.addRequirements(mIntake);
        this.setName("Default Command");
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        mIntake.intakeFeedFwd();
    }
    
}
