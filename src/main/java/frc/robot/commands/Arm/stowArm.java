package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class stowArm extends CommandBase{
    private Arm mArm;

    public stowArm() {
        mArm = Arm.getInstance();
        addRequirements(mArm);
        setName("Stow Arm");
    }

    @Override
    public void initialize() {
        mArm.setBrakeMode(false);
    }

    @Override
    public void execute() {
        mArm.setArmPosition(Constants.Arm.Positions.stowPosition);
    }

    @Override
    public void end(boolean interrupted) {
        mArm.setBrakeMode(true);
    }

}
