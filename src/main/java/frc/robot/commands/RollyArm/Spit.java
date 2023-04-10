package frc.robot.commands.RollyArm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hand;

public class Spit extends CommandBase {
    private Hand mHand = Hand.getInstance();
    public Spit() {
        this.setName("Spitting");
    }

    @Override
    public void initialize() {
        System.out.println("Rolly Claw Spitting!");
    }

    @Override
    public void execute() {
        mHand.Spit();
    }

    @Override
    public void end(boolean interrupted) {
        Hand.holding = false;
    }
}
