package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Vision;
import frc.robot.subsystems.Drivetrain;

public class Flatten extends CommandBase {
    Drivetrain mDrivetrain = Drivetrain.getInstance();
    Vision mVision = Vision.getInstance();
    double mSpeed;
    Vision.LimelightState mState;

    public Flatten(double speed, Vision.LimelightState state) {
        addRequirements(mDrivetrain);
        setName("Straighten Against Wall");
        mSpeed = speed;
        mState = state;
    }

    @Override
    public void execute() {
        if(mDrivetrain.getGyroAngle() < 90 && mDrivetrain.getGyroAngle() > 0) {
            mDrivetrain.spin(mSpeed);
        } else if(mDrivetrain.getGyroAngle() > -90 && mDrivetrain.getGyroAngle() < 0) {
            mDrivetrain.spin(-mSpeed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        mDrivetrain.spin(0);
    }

}
