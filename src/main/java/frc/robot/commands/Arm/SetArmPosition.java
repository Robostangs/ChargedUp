
package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Utils;
import frc.robot.Vision;
import frc.robot.subsystems.Arm;
public class SetArmPosition extends InstantCommand {

    private static Arm mArm = Arm.getInstance();
    private double distance = 0;
    private Arm.ArmPosition mDesiredState = null;
    private boolean mHolding = true;
    
    public SetArmPosition(Arm.ArmPosition state, boolean holding) {
        addRequirements(mArm);
        mDesiredState = state;
        mHolding = holding;
    }

    @Override
    public void initialize() { 
        distance = Vision.getInstance().getTargetHandX();
    }

    @Override
    public void execute() {
        System.out.println(mDesiredState);

            switch(mDesiredState) {
                case kStowPosition:
                    new ChangeSetPoint(new Utils.Vector2D(0.484, 0.3986)).schedule();
                    break;

                case kIntakePosition:
                    // new IntakingManager().schedule();
                    new ChangeSetPoint(new Utils.Vector2D(0.65, -0.4)).schedule();;
                    break;

                case kLoadingZonePosition:
                    new ChangeSetPoint(new Utils.Vector2D(0.62, 0.951)).schedule();
                    break;

                    
                case kLowPosition:
                    new ChangeSetPoint(new Utils.Vector2D(0.7, 0.158)).schedule();
                    break;

                case kMediumPosition:
                    if(mHolding) {
                        new ChangeSetPoint(new Utils.Vector2D(1.032, 1.127)).schedule();
                        System.out.println(mHolding);
                        System.out.println("Cone");
                    } else {
                        new ChangeSetPoint(new Utils.Vector2D(1.035, 0.752)).schedule();
                        System.out.println(mHolding);
                        System.out.println("Cube");
                    }
                    break;
                    
                case kHighPosition:
                    if(mHolding) {
                        new ChangeSetPoint(new Utils.Vector2D(0.6, 1.432)).withTimeout(1).andThen(
                        new ChangeSetPoint(new Utils.Vector2D(1.464, 1.432))).schedule();
                    } else {
                        new ChangeSetPoint(new Utils.Vector2D(0.6, 1.04)).withTimeout(1).andThen(
                        new ChangeSetPoint(new Utils.Vector2D(1.531, 1.04))).schedule();;
                    }
                    break;
            }
    }
}
