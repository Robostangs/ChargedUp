package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.LoggyThings.LoggyCANSparkMax;
import frc.robot.Constants;

public class Arm extends SubsystemBase{
    private static Arm mInstance;
    private LoggyCANSparkMax mArmMotor;
    private SparkMaxPIDController pidController;

    public static Arm getInstance() {
        if(mInstance == null) {
            mInstance = new Arm();
        }
        return mInstance;
    }

    public Arm() {
        mArmMotor = new LoggyCANSparkMax(Constants.Arm.armMotorID, CANSparkMaxLowLevel.MotorType.kBrushless, "/Arm/");
        pidController = mArmMotor.getPIDController();

        pidController.setP(Constants.Arm.armMotorP);
        pidController.setI(Constants.Arm.armMotorI);
        pidController.setD(Constants.Arm.armMotorD);
        pidController.setFF(Constants.Arm.armMotorFF);
        pidController.setIZone(Constants.Arm.armMotorI, 0);

        mArmMotor.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Arm Position", this::getArmPosition, this::setArmMotorPosition);
    }

    public void setArmMotor(double speed) {
        mArmMotor.set(speed);
    }

    public void setArmMotorPosition(double position) {
        mArmMotor.set(position);
    }

    public double getArmPosition() {
        return mArmMotor.getEncoder().getPosition();
    }
}
