package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * @deprecated
 * Use {@link PercentOutput} instead
 */

public class FineAdjust extends CommandBase {
    // // This class is needed because some arm movements needed to be treated as
    // // compound operations to avoid self-intersection.
    // private static Arm mArm = Arm.getInstance();
    // private DoubleSupplier mXOffset;
    // private DoubleSupplier mYOffset;
    // public FineAdjust(DoubleSupplier xOffset, DoubleSupplier yOffset) {
    //     mXOffset = xOffset;
    //     mYOffset = yOffset;
    //     addRequirements(mArm);
    //     setName("FineAdjust");
    // }

    // @Override
    // public void execute() {
    //     new ChangeSetPoint(new Vector2D(ChangeSetPoint.mCurrentSetpoint.x +mXOffset.getAsDouble(), ChangeSetPoint.mCurrentSetpoint.y + mYOffset.getAsDouble()));
    // }
}
