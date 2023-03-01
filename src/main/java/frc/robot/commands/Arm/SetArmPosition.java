
package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Utils;
import frc.robot.Vision;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Hand;

public class SetArmPosition extends SequentialCommandGroup {

    private static Arm mArm = Arm.getInstance();
    private double distance = 0;
    private Arm.ArmPosition mDesiredState = null;
    private boolean mHolding = true;

    public SetArmPosition(Arm.ArmPosition state, boolean holding) {
        addRequirements(mArm);
        mDesiredState = state;
        mHolding = holding;

        switch (mDesiredState) {
            case kStowPosition:
                addCommands(
                        new ChangeSetPoint(new Utils.Vector2D(0.59, 0.45)),
                        new WaitCommand(0.3),
                        new ChangeSetPoint(new Utils.Vector2D(0.59, 0.34)));
                break;

            case kIntakePosition:
                // new IntakingManager().schedule();
                addCommands(
                        new ChangeSetPoint(new Utils.Vector2D(0.65, -0.3)));
                break;

            case kLoadingZonePosition:
                addCommands(new ChangeSetPoint(new Utils.Vector2D(0.83, 0.99)));
                break;

            case kLowPosition:
                addCommands(new ChangeSetPoint(new Utils.Vector2D(0.7, 0.158)));
                break;

            case kMediumPosition:
                if (Hand.getInstance().getHolding()) {
                    addCommands(new ChangeSetPoint(new Utils.Vector2D(1.032, 1.127)));
                } else {
                    addCommands(new ChangeSetPoint(new Utils.Vector2D(1.035, 0.752)));
                }
                break;

            case kHighPosition:
                if (Hand.getInstance().getHolding()) {
                    addCommands(
                        new ChangeSetPoint(new Utils.Vector2D(0.6, 1.40)),
                        new WaitCommand(0.3),
                        new ChangeSetPoint(new Utils.Vector2D(1.485, 1.40)));
                } else {
                    addCommands(
                        new ChangeSetPoint(new Utils.Vector2D(0.6, 0.97)),
                        new WaitCommand(0.3),
                        new ChangeSetPoint(new Utils.Vector2D(1.531, 0.97)));
                }
                break;
        }
    }
}
