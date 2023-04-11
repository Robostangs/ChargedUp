package frc.robot.commands.RollyArm.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hand.Intake;

public class Spit extends CommandBase {
    private final Intake mIntake = Intake.getInstance();

    /**
     * Reverse Intake to clear the intake Mechanism
     */
    public Spit() {
        this.addRequirements(mIntake);
        this.setName("Spitting");
    }

    @Override
    public void initialize() {
        System.out.println("Rolly Claw Spitting!");
    }

    @Override
    public void execute() {
        mIntake.Spit();
    }

    @Override
    public void end(boolean interrupted) {
        // mIntake.setHolding(false);
    }
}
