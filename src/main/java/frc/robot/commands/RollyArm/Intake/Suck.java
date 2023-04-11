package frc.robot.commands.RollyArm.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hand.Intake;

public class Suck extends CommandBase {
    private final Intake mIntake = Intake.getInstance();

    /**
     * Intake to load the intake Mechanism
     */
    public Suck() {
        this.addRequirements(mIntake);
        this.setName("Sucking");
    }

    @Override
    public void initialize() {
        System.out.println("Rolly Claw Sucking!");
    }

    @Override
    public void execute() {
        mIntake.Suck();
        
    }

    @Override
    public void end(boolean interrupted) {
        // mIntake.setHolding(true);
    }
}
