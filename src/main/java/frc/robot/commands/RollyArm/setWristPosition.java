package frc.robot.commands.RollyArm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Hand;

public class setWristPosition extends InstantCommand {
    private final Hand mHand = Hand.getInstance();
    private double desiredAngle;
    private double rotation;

    public setWristPosition(double angle) {
        this.addRequirements(mHand);
        this.setName("Set Wrist to: " + angle + " Degrees");
        desiredAngle = angle;
    }

    @Override
    public void initialize() {
        // double desiredAngle = currentAngle + rotation;
        rotation = desiredAngle - mHand.getWristAngle();
        System.out.println("Wrist is going to move: " + rotation + " Degrees");
    }

    @Override
    public void execute() {
        mHand.setWristPosition(desiredAngle);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.out.println("Current Wrist Position: " + mHand.getWristAngle() + " Degrees");
        }
    }
}
