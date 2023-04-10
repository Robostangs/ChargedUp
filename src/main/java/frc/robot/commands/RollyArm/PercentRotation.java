package frc.robot.commands.RollyArm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Utils;
import frc.robot.subsystems.Hand;

public class PercentRotation extends CommandBase {
    private final Hand mHand = Hand.getInstance();
    double output;
    DoubleSupplier input;
    // Import Cone and Cube weight as they will factor in
    // Will be default command

    public PercentRotation(DoubleSupplier input) {
        this.setName("Rotating Claw");
        this.addRequirements(mHand);
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
        mHand.rawPower(output);
    }
}
