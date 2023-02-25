
package frc.robot.commands.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Vision;
import frc.robot.autos.rotation;
import frc.robot.subsystems.Swerve;

public class Flatten extends CommandBase {
    Swerve mDrivetrain = Swerve.getInstance();
    Vision mVision = Vision.getInstance();
    private double mAngle = 0;
    PIDController pidController = new PIDController(0.01, 0, 0.002);

    public Flatten() {
        addRequirements(mDrivetrain);
        setName("Straighten Against Wall");
    }

    public Flatten(double angle) {
        if(angle < 0.8) {
            end(false);
        }
        mAngle = angle;
        addRequirements(mDrivetrain);
        setName("Straighten Against Value");
        System.out.println("Im here part 3");
    }

    @Override
    

    public void execute() {
        //Path Stuff with the Mangle
        new rotation(mDrivetrain, mAngle);
    }
}
