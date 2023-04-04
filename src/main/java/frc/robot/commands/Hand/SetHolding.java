package frc.robot.commands.Hand;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Hand;

public class SetHolding extends InstantCommand{
    private Hand mHand = Hand.getInstance();
    private XboxController mManipController = RobotContainer.mManipController;
    private Boolean holding;
    /**
     * @deprecated
     * Use {@link ToggleHolding} instead
     */
    public SetHolding(Boolean holding) {
        this.holding = holding;
        addRequirements(mHand);
    }

    @Override
    public void initialize() {
        mHand.setHolding(holding);

        if(holding) {
            mManipController.setRumble(RumbleType.kLeftRumble, 0.25);
        } else {
            mManipController.setRumble(RumbleType.kRightRumble, 0.25);
        }
    }
    
}
