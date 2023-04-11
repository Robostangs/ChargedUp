package frc.robot.commands.RollyHand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hand;
import frc.robot.subsystems.RollyHand;

public class Hold extends CommandBase {
    public Hold(){
        addRequirements(RollyHand.getInstance());
    }
    public void execute() {
        RollyHand.getInstance().setRollerPower(.15);
    }
}
