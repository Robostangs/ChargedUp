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
import frc.robot.Utils.Vector2D;
import frc.robot.Vision;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Hand;

public class SetArmPosition extends SequentialCommandGroup {

    /**
         *
         */
        private static final Vector2D cubeHighPosition = new Utils.Vector2D(1.531, 1.13);
/**
         *
         */
        private static final Vector2D cubeHighTweenPosition = new Utils.Vector2D(0.6, 1.13);
/**
         *
         */
        private static final Vector2D coneHighPosition = new Utils.Vector2D(1.645, 1.56);
/**
         *
         */
        private static final Vector2D coneHighTweenPosition = new Utils.Vector2D(0.6, 1.56);
/**
         *
         */
        private static final Vector2D cubeMediumPosition = new Utils.Vector2D(1.035, 0.752);
/**
         *
         */
        private static final Vector2D cubeMediumTweenPosition = new Utils.Vector2D(0.59, 0.752);
/**
         *
         */
        private static final Vector2D coneMediumPosition = new Utils.Vector2D(1.032, 1.007);
/**
         *
         */
        private static final Vector2D coneMediumTweenPosition = new Utils.Vector2D(0.59, 1.17);
/**
         *
         */
        private static final Vector2D lowPosition = new Utils.Vector2D(0.7, 0.158);
/**
         *
         */
        private static final Vector2D loadingZonePosition = new Utils.Vector2D(0.7896, 0.995);
/**
         *
         */
        private static final Vector2D upIntakePosition = new Utils.Vector2D(0.65, -0.06);
/**
         *
         */
        private static final Vector2D generalIntakePosition = new Utils.Vector2D(0.65, -0.15);
/**
         *
         */
        private static final Vector2D stowPosition = new Utils.Vector2D(0.59, 0.34);
/**
         *
         */
        private static final Vector2D stowTweenPosition = new Utils.Vector2D(0.59, 0.45);
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
                        new ChangeSetPoint(stowTweenPosition).withTimeout(0.75),
                        new ChangeSetPoint(stowPosition));
                break;

            case kIntakePositionGeneral:
                // new IntakingManager().schedule();
                addCommands(
                        new InstantCommand(() -> SmartDashboard.putString("ArmPosition", mDesiredState.name())),
                        new ChangeSetPoint(generalIntakePosition));
                break;

            case kIntakePositionUp:
                addCommands(
                        new InstantCommand(() -> SmartDashboard.putString("ArmPosition", mDesiredState.name())),
                        new ChangeSetPoint(upIntakePosition));
                break;

            case kLoadingZonePosition:
                addCommands(
                        new InstantCommand(() -> SmartDashboard.putString("ArmPosition", mDesiredState.name())),
                        new ChangeSetPoint(loadingZonePosition));
                break;

            case kLowPosition:
                addCommands(
                        new InstantCommand(() -> SmartDashboard.putString("ArmPosition", mDesiredState.name())),
                        new ChangeSetPoint(lowPosition));
                break;

            case kMediumPosition:
                addCommands(new ConditionalCommand(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> SmartDashboard.putString("ArmPosition", mDesiredState.name())),
                                new ChangeSetPoint(coneMediumTweenPosition).withTimeout(2.5),
                                new ChangeSetPoint(coneMediumPosition)),
                        new SequentialCommandGroup(
                                new InstantCommand(() -> SmartDashboard.putString("ArmPosition", mDesiredState.name())),
                                new ChangeSetPoint(cubeMediumTweenPosition).withTimeout(2.5),
                                new ChangeSetPoint(cubeMediumPosition)),
                        () -> mHand.getHolding()));
                break;

            case kHighPosition:
                addCommands(new InstantCommand(() -> SmartDashboard.putBoolean("HoldingCone", mHand.holdingCone))
                        .andThen(new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new InstantCommand(
                                                () -> SmartDashboard.putString("ArmPosition", mDesiredState.name())),

                                        new PrintCommand("cone"),
                                        new ChangeSetPoint(coneHighTweenPosition).withTimeout(3),
                                        new ChangeSetPoint(coneHighPosition)),
                                new SequentialCommandGroup(
                                        new InstantCommand(
                                                () -> SmartDashboard.putString("ArmPosition", mDesiredState.name())),

                                        new PrintCommand("cube"),
                                        new ChangeSetPoint(cubeHighTweenPosition).withTimeout(3),
                                        new ChangeSetPoint(cubeHighPosition)),
                                () -> mHand.holdingCone)));
                break;
        }
    }
}
