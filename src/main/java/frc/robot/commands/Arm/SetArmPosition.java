package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Utils;
import frc.robot.subsystems.Arm;
public class SetArmPosition extends CommandBase{

    private static Arm mArm = Arm.getInstance();
    private static Arm.ArmPosition mDesiredState = null;
    private static Arm.ArmPosition mCurrentState = null;
    
    public SetArmPosition(Arm.ArmPosition state) {
        addRequirements(mArm);
        mDesiredState = state;
    }

    @Override
    public void initialize() { }

    @Override
    public void execute() {
        if(mDesiredState != mCurrentState) {
            switch(mDesiredState) {
                case kStowPosition:
                    new ChangeSetPoint(new Utils.Vector2D(0, 0)).schedule();
                    break;
                case kIntakePosition:
                    new ChangeSetPoint(new Utils.Vector2D(0, 0)).schedule();
                    break;
                case kLoadingZonePosition:
                    new ChangeSetPoint(new Utils.Vector2D(0, 0)).schedule();
                    break;
                case kLowPosition:
                    new ChangeSetPoint(new Utils.Vector2D(0, 0)).schedule();
                    break;
                case kMediumPosition:
                    new ChangeSetPoint(new Utils.Vector2D(0, 0)).schedule();
                    break;
                case kHighPosition:
                    new ChangeSetPoint(new Utils.Vector2D(0, 0)).schedule();
                    break;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        mCurrentState = mDesiredState;
    }
}
