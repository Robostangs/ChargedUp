package frc.robot.commands.Arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmState;

public class ArmStupid extends CommandBase{

    private static Arm mArm = Arm.getInstance();
    private static DoubleSupplier mShoulderSupplier;
    private static DoubleSupplier mElbowSupplier;

    public ArmStupid(DoubleSupplier ShoulderSupplier, DoubleSupplier ElbowSupplier) {
        addRequirements(mArm);
        mShoulderSupplier = ShoulderSupplier;
        mElbowSupplier = ElbowSupplier;
    }

    @Override
    public void initialize() {
        mArm.setBrakeMode(false);
    }

    @Override
    public void execute() {
        mArm.setElbowMotorPower(mElbowSupplier.getAsDouble());
        mArm.setShoulderMotorPower(mShoulderSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        mArm.setBrakeMode(true);
    }


}
