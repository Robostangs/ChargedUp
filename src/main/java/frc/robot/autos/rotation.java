package frc.robot.autos;

import frc.robot.Vision;
import frc.robot.subsystems.Swerve;

import javax.print.attribute.standard.MediaSize.Engineering;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class rotation extends CommandBase {
    Swerve mSwerve;
    Vision mVision = Vision.getInstance();
    boolean isReversed = false;
    double offset = 0;
    PIDController pid = new PIDController(0.3, 0,0 );

    double startAngle = 0;
    double endAngle = 0;

    public rotation(double rotation) {
        offset = rotation;
        mSwerve = Swerve.getInstance();
        addRequirements(Swerve.getInstance());
    }

    @Override
    public void initialize() {
        double gyroAngle = mSwerve.getRawGyroAngle();
        endAngle = gyroAngle + offset;
    }

    @Override
    public void execute() {
        double gyroAngle = mSwerve.getRawGyroAngle();
        startAngle = gyroAngle;
        // if (isReversed) {
        //     mSwerve.drive(new Translation2d(0, 0), (pid.calculate((360 + startAngle), endAngle + 360)), false, true);
        // } else {
        
        mSwerve.drive(new Translation2d(0, 0), (pid.calculate(startAngle, endAngle)), false, false);

        System.out.println(endAngle - startAngle);
        // }
    }

    @Override
    public boolean isFinished() {
        double gyroAngle = mSwerve.getRawGyroAngle();
        System.out.println(startAngle + "," + endAngle + "," + mSwerve.getGyroAngle());
        if (startAngle-endAngle <= 1) {
            return true;
        }
        return false;
    }
}
