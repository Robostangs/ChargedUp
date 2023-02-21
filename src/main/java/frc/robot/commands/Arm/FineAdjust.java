package frc.robot.commands.Arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class FineAdjust extends CommandBase {
    // This class is needed because some arm movements needed to be treated as
    // compound operations to avoid self-intersection.
    private static Arm mArm = Arm.getInstance();
    private DoubleSupplier mElbowAdjust;
    private DoubleSupplier mShoulderAdjust;
    
    public FineAdjust(DoubleSupplier elbowAdjust, DoubleSupplier shoulderAdjust) {
        mElbowAdjust = elbowAdjust;
        mShoulderAdjust = shoulderAdjust;
        addRequirements(mArm);
    }

    @Override
    public void execute() {
        mArm.offsetElbow(mElbowAdjust.getAsDouble() * 0.1);
        mArm.offsetShoulder(mShoulderAdjust.getAsDouble() * 0.1);
    }
}
