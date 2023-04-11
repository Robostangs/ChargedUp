package frc.robot.commands.RollyArm.Wrist;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Hand.Wrist;

public class setWristPosition extends InstantCommand {
    private final Wrist mWrist = Wrist.getInstance();
    private double desiredAngle;
    private double rotation;

    /**
     * Wrist Setpoints
     * @param angle the Desired angle you would like the arm to be at
     */
    public setWristPosition(double angle) {
        this.addRequirements(mWrist);
        this.setName("Set Wrist to: " + angle + " Degrees");
        desiredAngle = angle;
    }

    @Override
    public void initialize() {
        // double desiredAngle = currentAngle + rotation;
        rotation = desiredAngle - mWrist.getWristAngle();
        System.out.println("Wrist is going to move: " + rotation + " Degrees");
    }

    @Override
    public void execute() {
        mWrist.setWristPosition(desiredAngle);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.out.println("Current Wrist Position: " + mWrist.getWristAngle() + " Degrees");
        }
    }
}
