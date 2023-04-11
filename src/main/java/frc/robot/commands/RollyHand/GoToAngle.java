package frc.robot.commands.RollyHand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RollyHand;

public class GoToAngle extends CommandBase {
    double position;

    public GoToAngle(double p) {
        position = p;
        addRequirements(RollyHand.getInstance());
    }
    public void execute() {
        RollyHand.getInstance().setRotatorAngle(position);
    }
}
