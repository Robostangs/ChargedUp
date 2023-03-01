package frc.robot.commands.Arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class PercentOutput extends CommandBase {
    double shoulder, elbow;
    private Arm mArm = Arm.getInstance();

    public PercentOutput(DoubleSupplier shoulder, DoubleSupplier elbow) {
        this.shoulder = shoulder.getAsDouble();
        this.elbow = elbow.getAsDouble();
    }

    @Override
    public void initialize() {
        System.out.println("Shoulder: " + shoulder);
        System.out.println("Elbow: " + elbow);
    }

    @Override
    public void execute() {
        mArm.setPower(shoulder, elbow);
    }
}
