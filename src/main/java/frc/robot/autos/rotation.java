package frc.robot.autos;

import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class rotation extends CommandBase {
    Swerve mSwerve;
    PIDController pid = new PIDController(
        0.15, 0.12, 0.07);

    double startAngle = 0;
    double endAngle = 0;
    public rotation(Swerve swerve, double rotation){
        mSwerve = swerve;
        endAngle = rotation;  
        addRequirements(mSwerve);
              
    }

    @Override
    public void execute() {
        startAngle = mSwerve.getGyroAngle();
        mSwerve.drive(new Translation2d(0, 0), pid.calculate(startAngle, endAngle), false, true);
    }
}