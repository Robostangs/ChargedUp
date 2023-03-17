package frc.robot.commands.Swerve;

import frc.robot.Constants;
import frc.robot.Utils;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {    
    public static Swerve s_Swerve = Swerve.getInstance();    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier mSlowDown;

    public TeleopSwerve(DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, BooleanSupplier slowDown) {
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        mSlowDown = slowDown;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = Utils.customDeadzone(translationSup.getAsDouble());
        double strafeVal = Utils.customDeadzone(strafeSup.getAsDouble());
        double rotationVal = Utils.customDeadzone(rotationSup.getAsDouble());

        if(mSlowDown.getAsBoolean()) {
            translationVal *= 0.2;
            strafeVal *= 0.2;
            rotationVal *= 0.2;
        }

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
}