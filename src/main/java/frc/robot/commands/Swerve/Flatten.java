
package frc.robot.commands.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Vision;
import frc.robot.subsystems.Swerve;

public class Flatten extends CommandBase {
    Swerve mDrivetrain = Swerve.getInstance();
    Vision mVision = Vision.getInstance();
    private double mAngle = 0;
    PIDController pidController = new PIDController(0.01, 0, 0.002);
    double startAngle;

    public Flatten() {
        addRequirements(mDrivetrain);
        setName("Straighten Against Wall");
    }

    public Flatten(double angle) {
        mAngle = angle;
        addRequirements(mDrivetrain);
        setName("Straighten Against Value");
    }

    @Override
    public void initialize() {
        startAngle = mDrivetrain.getGyroAngle();
    }

    @Override
    public void execute() {
        //Path Stuff with the Mangle
        
        if(startAngle < 180 && startAngle > 00) {
            mDrivetrain.drive(new Translation2d(0, 0), -pidController.calculate(mDrivetrain.getGyroAngle(), 0) *10, false, true);
        } else if(startAngle > 180 && startAngle < 360) {
            mDrivetrain.drive(new Translation2d(0, 0), pidController.calculate(mDrivetrain.getGyroAngle(), 0) *10, false, true);
        } else {
            end(false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        mDrivetrain.drive(new Translation2d(0, 0), 0, false, false);
    }

}
