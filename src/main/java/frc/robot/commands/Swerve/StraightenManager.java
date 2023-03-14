package frc.robot.commands.Swerve;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Vision;
import frc.robot.subsystems.Swerve;

public class StraightenManager extends SequentialCommandGroup {
    Swerve mDrivetrain = Swerve.getInstance();
    Vision mVision = Vision.getInstance();

    public StraightenManager(double moveSpeed, double angleSpeed, boolean isCone) {
        addRequirements(mDrivetrain);
        setName("Straighten Against Wall");

        addCommands(
            new Flatten(angleSpeed),
            new GetToPosition(moveSpeed, isCone)
        );
    }

    
    public StraightenManager(boolean isCone) {
        addRequirements(mDrivetrain);
        setName("Straighten Against Wall");

        addCommands(
            // new Flatten(0),
            new InstantCommand(()-> DataLogManager.log("starting straighten")),
            new GetToPosition(0.2, isCone)
        );
    }


}
