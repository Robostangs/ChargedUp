package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Vision;
import frc.robot.Utils.Vector2D;
import frc.robot.commands.Hand.SetGrip;
import frc.robot.commands.Swerve.Flatten;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Arm.ArmPosition;

public class IntakingManager extends SequentialCommandGroup {
    private static final Arm mArm = Arm.getInstance();
    private static final Swerve mDrivetrain = Swerve.getInstance();
    private static final Vision mVision = Vision.getInstance();

    private double mTX = 0;
    private double mTargetHandX = 0;

    public IntakingManager() {
        mTX = mVision.getDrivetrainAngle();
        if(mTX < 0) {
            mTX = 360 - mTX;
        }

        mTargetHandX = mVision.getTargetHandX();
        addRequirements(mArm, mDrivetrain);

        // new ChangeSetPoint(new Vector2D(0.7, -0.07));
    
        addCommands(
            // new SetGrip(),
            // new Flatten(mTX).andThen(() -> System.out.println("here part 2")),
            new ChangeSetPoint(new Vector2D(mTargetHandX, 0))
            // new SetGrip(),
            // new SetArmPosition(ArmPosition.kStowPosition, true)
        );
    }
    
}
