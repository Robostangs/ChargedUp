package frc.robot.commands.RollyHand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RollyHand;

public class Throw extends CommandBase {
    public Throw(){
        addRequirements(RollyHand.getInstance());

    }
    public void execute() {
        RollyHand.getInstance().setRollerPower(-1.0);
    }
}
