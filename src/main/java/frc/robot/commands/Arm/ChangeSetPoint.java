package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Utils;
import frc.robot.subsystems.Arm;

public class ChangeSetPoint extends CommandBase {
    // This class is needed because some arm movements needed to be treated as
    // compound operations to avoid self-intersection.
    private static Arm mArm = Arm.getInstance();
    private static Utils.Vector2D setPoint;
    
    public ChangeSetPoint(Utils.Vector2D s) {
        setPoint = s;
        addRequirements(mArm);
    }

    @Override
    public void execute() {
        mArm.changeSetpoint(setPoint);
    }
}
