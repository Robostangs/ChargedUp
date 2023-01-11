package frc.robot.commands.Drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Utils;
import frc.robot.subsystems.Drivetrain;

public class ArcadeDrive extends CommandBase{
    
    private Drivetrain mDrivetrain;
    private DoubleSupplier mXSupplier, mYSupplier, mTranslationSupplier;

    public ArcadeDrive(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier translationSupplier) {
        mXSupplier = xSupplier;
        mYSupplier = ySupplier;
        mTranslationSupplier = translationSupplier;
        mDrivetrain = Drivetrain.getInstance();
        addRequirements(mDrivetrain);
    }

    @Override
    public void execute() {
        mDrivetrain.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                Utils.customDeadzone(mXSupplier.getAsDouble()), 
                Utils.customDeadzone(mYSupplier.getAsDouble()), 
                mTranslationSupplier.getAsDouble(), 
                new Rotation2d(mDrivetrain.getGyroAngle())
            )
        );
    }

}
