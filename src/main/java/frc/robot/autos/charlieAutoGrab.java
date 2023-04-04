package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.LoggyThings.LoggyPrintCommand;
import frc.robot.Constants;
import frc.robot.commands.Arm.ProfiledChangeSetPoint;
import frc.robot.commands.Hand.SetGrip;

public class charlieAutoGrab{
    public static SequentialCommandGroup getCommand() {
        new charlieAutoDriveToCube();
        return new SequentialCommandGroup(
            ProfiledChangeSetPoint.createWithTimeout(() -> Constants.Arm.SetPoint.generalIntakePosition).alongWith(new LoggyPrintCommand("First Part")),
            // new WaitCommand(1),
            charlieAutoDriveToCube.getCommand().deadlineWith(new SetGrip()).alongWith(new LoggyPrintCommand("SecondPart")),
            ProfiledChangeSetPoint.createWithTimeout(() -> Constants.Arm.SetPoint.stowPosition).alongWith(new LoggyPrintCommand("Third Part")));
    }
}

