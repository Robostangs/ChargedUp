package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Utils;
import frc.robot.Vision;
import frc.robot.subsystems.Arm;
public class SetArmPosition extends CommandBase{

    private static Arm mArm = Arm.getInstance();
    private double distance = 0;
    private Arm.ArmPosition mDesiredState = null;
    
    public SetArmPosition(Arm.ArmPosition state) {
        addRequirements(mArm);
        mDesiredState = state;
    }

    @Override
    public void initialize() { 
        distance = Vision.getInstance().getDrivetrainDistance();
    }

    @Override
    public void execute() {
        System.out.println(mDesiredState);

            switch(mDesiredState) {
                case kStowPosition:
                    new ChangeSetPoint(new Utils.Vector2D(0.2, 0.45)).schedule();
                    break;
                case kIntakePosition:
                    new IntakingManager().schedule();
                    break;
                case kLoadingZonePosition:
                    new ChangeSetPoint(new Utils.Vector2D(0, 0)).schedule();
                    break;
                case kLowPosition:
                    new ChangeSetPoint(new Utils.Vector2D(0.87, 0.16)).schedule();
                    break;
                case kMediumPositionCone:
                    new ChangeSetPoint(new Utils.Vector2D(1.1, 0.9)).schedule();
                    break;
                case kHighPositionCone:
                    new ChangeSetPoint(new Utils.Vector2D(1.45, 1.2)).schedule();
                    break;
                case kMediumPositionCube:
                    new ChangeSetPoint(new Utils.Vector2D(1.15, 1.0)).schedule();
                    break;
                case kHighPositionCube:
                    new ChangeSetPoint(new Utils.Vector2D(1.5, 1.0)).schedule();
                    break;
            }
    }   
}
