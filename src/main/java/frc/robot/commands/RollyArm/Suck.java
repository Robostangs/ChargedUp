package frc.robot.commands.RollyArm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hand;

public class Suck extends CommandBase {
    private Hand mHand = Hand.getInstance();
    public Suck() {
        this.setName("Sucking");
        this.addRequirements(mHand);
    }

    @Override
    public void initialize() {
        System.out.println("Rolly Claw Sucking!");
    }

    @Override
    public void execute() {
        mHand.Suck();
        
    }

    @Override
    public void end(boolean interrupted) {
        Hand.holding = true;
    }
}
