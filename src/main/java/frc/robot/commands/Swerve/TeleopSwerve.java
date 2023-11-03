package frc.robot.commands.Swerve;

import frc.robot.Constants;
import frc.robot.Utils;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopSwerve extends CommandBase {
    private Swerve s_Swerve = Swerve.getInstance();
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier mSlowDown;
    /*
     * Multiply the strafeVal by a certain modifier to get a ratio where the robot
     * will not rotate while moving. This ratio will be added/subtracted to the
     * rotationVal when rotationVal == 0
     * This is to prevent the robot from rotating when the driver is trying to
     * strafe.
     */
    private double rotationMod;
    private final double strafeRatio = 0.5;
    // private DoubleSupplier

    /**
     * Default Swerve Code
     * 
     * @param translationSup  (DoubleSupplier) Forward & Backwards
     * @param strafeSup       (DoubleSupplier) Left & Right
     * @param rotationSup     (DoubleSupplier) Turning angle
     * @param robotCentricSup (BooleanSupplier) Whether or not the robot drives from
     *                        its own POV
     * @param slowDown        (BooleanSupplier) Decrease speed for sensitive tasks
     */
    public TeleopSwerve(
            DoubleSupplier translationSup,
            DoubleSupplier strafeSup,
            DoubleSupplier rotationSup,
            BooleanSupplier robotCentricSup,
            BooleanSupplier slowDown) {
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        mSlowDown = slowDown;
        if (Math.abs(rotationSup.getAsDouble()) < 0.1) {
            rotationMod = strafeRatio * strafeSup.getAsDouble();
        } else {
            rotationMod = 0;
        }
    }

    @Override
    public void execute() {
        /* Get Values, Deadband */
        double translationVal = Utils.customDeadzone(translationSup.getAsDouble());
        double strafeVal = Utils.customDeadzone(strafeSup.getAsDouble());
        double rotationVal = Utils.customDeadzone(rotationSup.getAsDouble());

        if (mSlowDown.getAsBoolean()) {
            translationVal *= 0.2;
            strafeVal *= 0.2;
            rotationVal *= 0.2;
        }

        /* Drive */
        s_Swerve.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
                // (rotationVal - rotationMod) * Constants.Swerve.maxAngularVelocity,
                (rotationVal) * Constants.Swerve.maxAngularVelocity,
                !robotCentricSup.getAsBoolean(),
                true);
    }
}