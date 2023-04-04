package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Arm.ProfiledChangeSetPoint;
import frc.robot.commands.Hand.SetGrip;

public class charlieAutoGrab extends CommandBase {
    private boolean isFinished = false;
    public charlieAutoGrab() {     
        
    }

    @Override
    public void initialize() {
        new SequentialCommandGroup(ProfiledChangeSetPoint.createWithTimeout(() -> Constants.Arm.SetPoint.generalIntakePosition),
            // new WaitCommand(1),
            charlieAutoDriveToCube.getCommand().deadlineWith(new SetGrip()),
            ProfiledChangeSetPoint.createWithTimeout(() -> Constants.Arm.SetPoint.stowPosition), 
            new InstantCommand(() -> isFinished = true)).schedule();;
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}

