package frc.robot.commands.Arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class FineAdjust extends CommandBase {
    // This class is needed because some arm movements needed to be treated as
    // compound operations to avoid self-intersection.
    private static Arm mArm = Arm.getInstance();
    private DoubleSupplier mXOffset;
    private DoubleSupplier mYOffset;
    
    public FineAdjust(DoubleSupplier xOffset, DoubleSupplier yOffset) {
        mXOffset = xOffset;
        mYOffset = yOffset;
        addRequirements(mArm);
        setName("FineAdjust");
    }

    @Override
    public void execute() {
        mArm.offsetX(mXOffset.getAsDouble() * 0.02  *Constants.Arm.ManualAdjustMPS);
        mArm.offsetY(mYOffset.getAsDouble() * 0.02 * Constants.Arm.ManualAdjustMPS);
    }
}
