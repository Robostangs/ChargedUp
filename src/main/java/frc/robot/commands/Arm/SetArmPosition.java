package frc.robot.commands.Arm;

import java.time.Instant;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.LoggyThings.DashboardPutCommand;
import frc.LoggyThings.LoggyPrintCommand;
import frc.robot.Utils;
import frc.robot.Utils.Vector2D;
import frc.robot.Vision;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Hand;
import frc.robot.subsystems.Arm.ArmPosition;

public class SetArmPosition extends SequentialCommandGroup {

    //Scoring Positions
    private static final Vector2D cubeHighTweenPosition = new Utils.Vector2D(0.66, 1.24);

    private static final Vector2D cubeHighPosition = new Utils.Vector2D(1.731, 1.25);
    private static final Vector2D coneHighTweenPosition = new Utils.Vector2D(0.66, 1.50);
    private static final Vector2D coneHighPosition = new Utils.Vector2D(1.44, 1.3);

    private static final Vector2D cubeMediumTweenPosition = new Utils.Vector2D(0.65, 0.881);
    private static final Vector2D cubeMediumPosition = new Utils.Vector2D(1.067, 0.781);

    private static final Vector2D coneMediumTweenPosition = new Utils.Vector2D(0.65, 1.17);
    private static final Vector2D coneMediumPosition = new Utils.Vector2D(1.042, 0.987);

    private static final Vector2D lowPosition = new Utils.Vector2D(0.7, 0.158);

    //Intaking Positions
    private static final Vector2D loadingZonePosition = new Utils.Vector2D(0.666, 1.005);

    private static final Vector2D intakeTweenPosition = new Vector2D(0.75, 0.4);

    private static final Vector2D upIntakePosition = new Utils.Vector2D(0.59, 0.09);
    private static final Vector2D generalIntakePosition = new Utils.Vector2D(0.65, 0);

    //Stowing Positions
    private static final Vector2D highStowTweenPosition = new Utils.Vector2D(0.59, 1.3);
    private static final Vector2D stowTweenPosition = new Utils.Vector2D(0.59, 0.45);
    private static final Vector2D stowPosition = new Utils.Vector2D(0.423, 0.324);

    private static final Vector2D startTweenPosition = new Utils.Vector2D(0.6, 0.4);
    private static final Vector2D startPosition = new Utils.Vector2D(0.27, 0.18);

    private static Arm mArm = Arm.getInstance();
    private Hand mHand = Hand.getInstance();
    private Arm.ArmPosition mDesiredState = null;
    private static ArmPosition mPreviousPosition = ArmPosition.kHighPosition;

    public SetArmPosition(Arm.ArmPosition state) {
        addRequirements(mArm);
        mDesiredState = state;

        addCommands(new DashboardPutCommand("ArmPosition", mDesiredState.name()));
        switch (mDesiredState) {
            case kStartPosition:
                addCommands(
                        ChangeSetPoint.createWithTimeout(startTweenPosition),
                        ChangeSetPoint.createWithTimeout(startPosition));
                break;
            case kStowPosition:
                addCommands(
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        ChangeSetPoint.createWithTimeout(highStowTweenPosition),
                                        ChangeSetPoint.createWithTimeout(stowPosition)),
                                new SequentialCommandGroup(
                                        ChangeSetPoint.createWithTimeout(stowTweenPosition),
                                        ChangeSetPoint.createWithTimeout(stowPosition)),
                                () -> mPreviousPosition.equals(ArmPosition.kHighPosition)));
                break;

            case kIntakePositionGeneral:
                // new IntakingManager().schedule();
                addCommands(
                        ChangeSetPoint.createWithTimeout(intakeTweenPosition),
                        ChangeSetPoint.createWithTimeout(generalIntakePosition));
                break;

            case kIntakePositionUp:
                addCommands(
                        ChangeSetPoint.createWithTimeout(intakeTweenPosition),
                        ChangeSetPoint.createWithTimeout(upIntakePosition));

                break;

            case kLoadingZonePosition:
                addCommands(
                        ChangeSetPoint.createWithTimeout(loadingZonePosition));
                break;

            case kLowPosition:
                addCommands(ChangeSetPoint.createWithTimeout(lowPosition));
                break;

            case kMediumPosition:
                addCommands(
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        ChangeSetPoint.createWithTimeout(coneMediumTweenPosition),
                                        ChangeSetPoint.createWithTimeout(coneMediumPosition)),
                                new SequentialCommandGroup(
                                        ChangeSetPoint.createWithTimeout(cubeMediumTweenPosition),
                                        ChangeSetPoint.createWithTimeout(cubeMediumPosition)),
                                () -> mHand.getHolding()));
                break;

            case kHighPosition:
                addCommands(
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        ChangeSetPoint.createWithTimeout(coneHighTweenPosition),
                                        ChangeSetPoint.createWithTimeout(coneHighPosition)),
                                new SequentialCommandGroup(
                                        ChangeSetPoint.createWithTimeout(cubeHighTweenPosition),
                                        ChangeSetPoint.createWithTimeout(cubeHighPosition)),
                                () -> mHand.holdingCone));

                break;
        }
        addCommands(new InstantCommand(() -> mPreviousPosition = mDesiredState));
        addCommands(new LoggyPrintCommand("SET ARM POSITION DONE"));
        addCommands(new LoggyPrintCommand(mPreviousPosition));
    }
}
