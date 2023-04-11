package frc.robot.commands.RollyArm.Intake;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Hand.Intake;

public class ToggleHolding extends InstantCommand {

    private Intake mIntake = Intake.getInstance();
    private XboxController mManipController = RobotContainer.mManipController;

    /**
     * Change the Cone Holding State, does not affect Claw State
     */
    public ToggleHolding() {
        addRequirements(mIntake);
    }

    @Override
    public void initialize() {
        mIntake.setHolding(!mIntake.getHolding());
        
        if(mIntake.getHolding()) {
            mManipController.setRumble(RumbleType.kLeftRumble, 0.25);
        } else {
            mManipController.setRumble(RumbleType.kRightRumble, 0.25);
        }
    }
}
