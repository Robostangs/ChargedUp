package frc.robot.commands.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class balance extends CommandBase {
    private static final Swerve mDrivetrain = Swerve.getInstance();
    private static final PIDController balancePID = new PIDController(Constants.Swerve.balancePID.kP, Constants.Swerve.balancePID.kI, Constants.Swerve.balancePID.kD);

    /**
     * Receive input from Gyro and Maintain Robot Level at 0
     */
    public balance() {
        addRequirements(mDrivetrain);
        setName("Balance");
    }

    @Override
    public void execute() {
        mDrivetrain.drive(new Translation2d(-balancePID.calculate(0, mDrivetrain.getPitchAngle()) * 4.5, 0), 0, false, true);
    }

    @Override
    public void end(boolean interrupted) {
        mDrivetrain.drive(new Translation2d(0, 0), .25, false, false);
        balancePID.close();
    }

}
