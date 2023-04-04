package frc.robot.commands.Arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class PercentOutput extends CommandBase {
    DoubleSupplier shoulder, elbow;
    private Arm mArm = Arm.getInstance();

    /**
     * Manually adjust the {@link Arm}
     * @param shoulder DoubleSupplier for shoulder movement
     * @param elbow DoubleSupplier for elbow movement
     */
    public PercentOutput(DoubleSupplier shoulder, DoubleSupplier elbow) {
        addRequirements(mArm);
        setName("PercentOutput");
        this.shoulder = shoulder;
        this.elbow = elbow;
    }

    // @Override
    // public void initialize() {
    //     // System.out.println("Shoulder: " + shoulder);
    //     // System.out.println("Elbow: " + elbow);
    // }

    @Override
    public void execute() {
        mArm.setPower(-shoulder.getAsDouble(), elbow.getAsDouble());
    }
}
