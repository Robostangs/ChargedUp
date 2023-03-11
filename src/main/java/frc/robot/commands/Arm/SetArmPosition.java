package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
                        new InstantCommand(() -> SmartDashboard.putString("ArmPosition", mDesiredState.name())),
                        new ChangeSetPoint(new Utils.Vector2D(0.59, 0.45)).withTimeout(0.75),
                        new WaitCommand(0.4),
                        new ChangeSetPoint(new Utils.Vector2D(0.59, 0.34)));
                break;

            case kIntakePositionGeneral:
                // new IntakingManager().schedule();
                addCommands(
                        new InstantCommand(() -> SmartDashboard.putString("ArmPosition", mDesiredState.name())),
                        new ChangeSetPoint(new Utils.Vector2D(0.65, -0.15)));
                break;

            case kIntakePositionUp:
                addCommands(
                        new InstantCommand(() -> SmartDashboard.putString("ArmPosition", mDesiredState.name())),
                        new ChangeSetPoint(new Utils.Vector2D(0.65, -0.06)));
                break;

            case kLoadingZonePosition:
                addCommands(
                        new InstantCommand(() -> SmartDashboard.putString("ArmPosition", mDesiredState.name())),
                        new ChangeSetPoint(new Utils.Vector2D(0.7896, 0.995)));
                break;

            case kLowPosition:
                addCommands(
                        new InstantCommand(() -> SmartDashboard.putString("ArmPosition", mDesiredState.name())),
                        new ChangeSetPoint(new Utils.Vector2D(0.7, 0.158)));
                break;

            case kMediumPosition:
                addCommands(new ConditionalCommand(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> SmartDashboard.putString("ArmPosition", mDesiredState.name())),
                                new ChangeSetPoint(new Utils.Vector2D(0.59, 1.17)).withTimeout(2.5),
                                new WaitCommand(0.4),
                                new ChangeSetPoint(new Utils.Vector2D(1.032, 1.007))),
                        new SequentialCommandGroup(
                                new InstantCommand(() -> SmartDashboard.putString("ArmPosition", mDesiredState.name())),
                                new ChangeSetPoint(new Utils.Vector2D(0.59, 0.752)).withTimeout(2.5),
                                new WaitCommand(0.4),
                                new ChangeSetPoint(new Utils.Vector2D(1.035, 0.752))),
                        () -> mHand.getHolding()));
                break;

            case kHighPosition:
                addCommands(new InstantCommand(() -> SmartDashboard.putBoolean("HoldingCone", mHand.holdingCone))
                        .andThen(new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new InstantCommand(
                                                () -> SmartDashboard.putString("ArmPosition", mDesiredState.name())),

                                        new PrintCommand("cone"),
                                        new ChangeSetPoint(new Utils.Vector2D(0.6, 1.56)).withTimeout(3),
                                        new WaitCommand(0.5),
                                        new ChangeSetPoint(new Utils.Vector2D(1.645, 1.56))),
                                new SequentialCommandGroup(
                                        new InstantCommand(
                                                () -> SmartDashboard.putString("ArmPosition", mDesiredState.name())),

                                        new PrintCommand("cube"),
                                        new ChangeSetPoint(new Utils.Vector2D(0.6, 1.13)).withTimeout(3),
                                        new WaitCommand(0.5),
                                        new ChangeSetPoint(new Utils.Vector2D(1.531, 1.13))),
                                () -> mHand.holdingCone)));
                break;
        }
    }
}

