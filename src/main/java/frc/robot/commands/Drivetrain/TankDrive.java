package frc.robot.commands.Drivetrain;

import frc.robot.Constants;
import frc.robot.Utils;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TankDrive extends CommandBase {    
    private Swerve s_Swerve = Swerve.getInstance();    
    private DoubleSupplier mLeftSupplier;
    private DoubleSupplier mRightSupplier;
    private DoubleSupplier mAngleSupplier;

    public TankDrive(DoubleSupplier leftSupplier, DoubleSupplier rightSupplier, DoubleSupplier angleSupplier) {
        addRequirements(s_Swerve);

        mLeftSupplier = leftSupplier;
        mRightSupplier = rightSupplier;
        mAngleSupplier = angleSupplier;
    }

    @Override
    public void execute() {
        double leftPower = Utils.customDeadzone(mLeftSupplier.getAsDouble());
        double rightPower = Utils.customDeadzone(mRightSupplier.getAsDouble());
        double anglePower = mAngleSupplier.getAsDouble();

        s_Swerve.drive(new Translation2d(, ), anglePower, false, false);
    }
}