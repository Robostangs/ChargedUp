package frc.robot.commands.RollyArm.Wrist;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Utils;
import frc.robot.subsystems.Hand.Wrist;

public class PercentRotation extends CommandBase {
    private final Wrist mWrist = Wrist.getInstance();
    double output;
    DoubleSupplier input;
    // Import Cone and Cube weight as they will factor in
    // Will be default command

    /**
     * Manual Rotation of the Wrist
     * @param input XboxController Joystick that controlls angle movement of Wrist
     */
    public PercentRotation(DoubleSupplier input) {
        this.addRequirements(mWrist);
        this.setName("Rotating Claw");
        this.input = input;
    }

    @Override
    public void initialize() {
        System.out.println("Moving Arm: " + input.getAsDouble());
    }

    @Override
    public void execute() {
        // I need to put in how much is the max rotation
        // Also need a scale for how much input on xbox controller joystick equals how much angle change
        output = Utils.customDeadzone(input.getAsDouble());
        if (output == 0) {
            output = 0.1;
        }

        mWrist.rawPower(output);
    }
}
