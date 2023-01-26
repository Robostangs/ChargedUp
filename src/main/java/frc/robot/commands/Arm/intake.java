package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class intake extends CommandBase{
    private Arm mArm;

    public intake() {
        mArm = Arm.getInstance();
        addRequirements(mArm);
        setName("Move to Intake Position");
    }

    @Override
    public void initialize() {
        mArm.setBrakeMode(false);
    }

    @Override
    public void execute() {
        mArm.setArmPosition(Constants.Arm.Positions.intakePosition);
    }

    @Override
    public void end(boolean interrupted) {
        mArm.setBrakeMode(true);
    }

}
