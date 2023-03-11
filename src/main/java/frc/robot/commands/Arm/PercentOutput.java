package frc.robot.commands.Arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class PercentOutput extends CommandBase {
    DoubleSupplier shoulder, elbow;
    private Arm mArm = Arm.getInstance();

    public PercentOutput(DoubleSupplier shoulder, DoubleSupplier elbow) {
        addRequirements(mArm);
        this.shoulder = shoulder;
        this.elbow = elbow;
    }

    @Override
    public void initialize() {
        mArm.changeSetpoint(null,null);
        DataLogManager.log("Shoulder: " + shoulder);
        DataLogManager.log("Elbow: " + elbow);
    }

    @Override
    public void execute() {
        mArm.setPower(shoulder.getAsDouble(), elbow.getAsDouble());
    }
}
