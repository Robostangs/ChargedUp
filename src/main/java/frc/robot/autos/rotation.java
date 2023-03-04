package frc.robot.autos;

import frc.robot.Vision;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class rotation extends InstantCommand {
    Swerve mSwerve;
    Vision mVision = Vision.getInstance();
    boolean isReversed = false;
    double offset = 0;
    PIDController pid = new PIDController(0.15, 0.1, 0.12);

    double startAngle = 0;
    double endAngle = 0;

    public rotation(double rotation) {
        endAngle = rotation;
        mSwerve = Swerve.getInstance();
        addRequirements(Swerve.getInstance());
    }

    @Override
    public void initialize() {
        if (endAngle > 0) {
            isReversed = false;
        } else {
            isReversed = true;
        }

        offset = mSwerve.getGyroAngle();
        DataLogManager.log("pause");
    }

    @Override
    public void execute() {
        startAngle = mSwerve.getGyroAngle() - offset;
        if (isReversed) {
            mSwerve.drive(new Translation2d(0, 0), (pid.calculate((360 + startAngle), endAngle + 360)* Math.pow(10.1 / RobotController.getBatteryVoltage(), 0.7)), false, true);
        } else {
            mSwerve.drive(new Translation2d(0, 0), (pid.calculate(startAngle, endAngle) * Math.pow(10.1 / RobotController.getBatteryVoltage(), 0.7)), false, true);
        }
    }

    @Override
    public boolean isFinished() {
        System.out.println(startAngle + "," + endAngle + "," + mSwerve.getGyroAngle());
        if (Math.abs((mSwerve.getGyroAngle() - offset) - endAngle) <= 1) {
            return true;
        }
        return false;
    }
}
