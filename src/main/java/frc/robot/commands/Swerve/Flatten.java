package frc.robot.commands.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Vision;
import frc.robot.autos.rotation;
import frc.robot.subsystems.Swerve;

public class Flatten extends InstantCommand {
    Swerve mDrivetrain = Swerve.getInstance();
    Vision mVision = Vision.getInstance();
    private double mAngle = 0;
    PIDController pidController = new PIDController(0.01, 0, 0.002);

    /**
     * Flatten Robot against a wall
     */
    public Flatten() {
        addRequirements(mDrivetrain);
        setName("Straighten Against Wall");
    }

    /**
     * Flatten Robot to certain angle
     * @param angle what degree to set the Robot too
     */
    public Flatten(double angle) {
        if(angle < 0.8) {
            end(false);
        }
        mAngle = angle;
        addRequirements(mDrivetrain);
        setName("Straighten Against Value");
    }

    @Override
    public void execute() {
        //Path Stuff with the mAngle
        new rotation(mAngle);
    }
}
