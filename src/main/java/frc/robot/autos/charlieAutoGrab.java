package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Vision;
import frc.robot.Utils.Vector2D;
import frc.robot.Utils.Vector3D;
import frc.robot.commands.Arm.SetArmPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Arm.ArmPosition;

public class charlieAutoGrab extends CommandBase {
    public charlieAutoGrab() {
        addRequirements(Swerve.getInstance());

    }

    public double sin(double value) {
        return Math.sin(Math.toRadians(value));
    }

    public double cos(double value) {
        return Math.cos(Math.toRadians(value));
    }

    public void initialize() {
        // Vector2D mVector2d = Vision.getInstance().objectPosition(); 

        new SequentialCommandGroup(
            new SetArmPosition(ArmPosition.kIntakePositionGeneral),
            new WaitCommand(1),
            new rotation(-Vision.getInstance().getCenterLimelightTX()),
            new WaitCommand(1),
            new PrintCommand(new Vector2D(Math.abs(((sin(90+Vision.getInstance().objectPosition().y) * Constants.Arm.upperarmLength) / sin(90-Vision.getInstance().objectPosition().y-Arm.getInstance().getShoulderPositionFromMotor())))- (Arm.getInstance().getHandPositionX() -0.2) + 0.05, 0).toString()),
            new translateWithoutTrajectory(new Vector2D(Math.abs(((sin(90+Vision.getInstance().objectPosition().y) * Constants.Arm.upperarmLength) / sin(90-Vision.getInstance().objectPosition().y-Arm.getInstance().getShoulderPositionFromMotor())))- (Arm.getInstance().getHandPositionX() -0.2) + 0.05, 0))
        ).schedule();
    }
}
