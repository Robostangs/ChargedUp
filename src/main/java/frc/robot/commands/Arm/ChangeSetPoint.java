package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Utils;
import frc.robot.subsystems.Arm;

public class ChangeSetPoint extends CommandBase {
    // This class is needed because some arm movements needed to be treated as
    // compound operations to avoid self-intersection.
    private Arm mArm = Arm.getInstance();
    private Utils.Vector2D setPoint;
    private boolean hasRun = false;
    private int repeating;

    public ChangeSetPoint(Utils.Vector2D s) {
        setPoint = s;
        addRequirements(mArm);
        setName("ChangeSetPoint");
    }

    @Override
    public void initialize() {
        // repeating = 0;
        // hasRun = false;
        // mArm.changeSetpoint(setPoint);
    }

    @Override
    public void execute() {
        // if(repeating >= 20) {
        //     hasRun = true;
        // }
        // repeating++;
    }

    @Override
    public boolean isFinished() {
        // if (mArm.getElbowLocked() && mArm.getShoulderLocked() && hasRun) {
        //     System.out.println("changeSetPoint: finished");
        //     return true;
        // }
        // return false;
        return true;
    }
}
