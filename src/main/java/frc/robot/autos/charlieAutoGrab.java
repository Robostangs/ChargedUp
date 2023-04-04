package frc.robot.autos;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.Arm.ProfiledChangeSetPoint;
import frc.robot.commands.Hand.SetGrip;

public class charlieAutoGrab extends CommandBase {
    private boolean a = false;
    public charlieAutoGrab() {     
        
    }

    @Override
    public void initialize() {
        new SequentialCommandGroup(ProfiledChangeSetPoint.createWithTimeout(() -> Constants.Arm.SetPoint.generalIntakePosition),
            // new WaitCommand(1),
            charlieAutoDriveToCube.getCommand().deadlineWith(new SetGrip()),
            ProfiledChangeSetPoint.createWithTimeout(() -> Constants.Arm.SetPoint.stowPosition), 
            new InstantCommand(() -> a = true)).schedule();;
    }

    @Override
    public boolean isFinished() {
        return a;
    }
}
