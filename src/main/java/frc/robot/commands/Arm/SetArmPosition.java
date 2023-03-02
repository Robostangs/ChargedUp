package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Utils;
import frc.robot.Vision;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Hand;

public class SetArmPosition extends SequentialCommandGroup {

    private static Arm mArm = Arm.getInstance();
    private Hand mHand = Hand.getInstance();
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
                        new WaitCommand(0.5),
                        new ChangeSetPoint(new Utils.Vector2D(0.59, 0.34)));
                break;

            case kIntakePositionGeneral:
                // new IntakingManager().schedule();
                addCommands(
                    new ChangeSetPoint(new Utils.Vector2D(0.65, -0.3)));
                break;

            case kIntakePositionUp:
                addCommands(
                    new ChangeSetPoint(new Utils.Vector2D(0.65, -0.1)));
                break;

            case kLoadingZonePosition:
                addCommands(new ChangeSetPoint(new Utils.Vector2D(0.83, 0.995)));
                break;

            case kLowPosition:
                addCommands(new ChangeSetPoint(new Utils.Vector2D(0.7, 0.158)));
                break;

            case kMediumPosition:
                addCommands(new ConditionalCommand(
                    new ChangeSetPoint(new Utils.Vector2D(1.032, 1.127)),
                    new ChangeSetPoint(new Utils.Vector2D(1.035, 0.602)),
                    () -> mHand.getHolding()));
                break;

            case kHighPosition:
                addCommands(new InstantCommand(() -> System.out.println(mHand.holdingCone))
                    .andThen(new ConditionalCommand(
                        new SequentialCommandGroup(
                            new PrintCommand("cone"),
                            new ChangeSetPoint(new Utils.Vector2D(0.6, 1.48)),
                            new WaitCommand(0.5),
                            new ChangeSetPoint(new Utils.Vector2D(1.485, 1.48))),
                        new SequentialCommandGroup(
                            new PrintCommand("cube"),
                            new ChangeSetPoint(new Utils.Vector2D(0.6, 0.88)),
                            new WaitCommand(0.5),
                            new ChangeSetPoint(new Utils.Vector2D(1.531, 0.88))),
                        () -> mHand.holdingCone
                    )));
                break;
        }
    }
}

