package frc.robot.commands.Arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Utils;
import frc.robot.subsystems.Arm;

public class temp_ChangeSetPoint extends CommandBase {
    // This class is needed because some arm movements needed to be treated as
    // compound operations to avoid self-intersection.
    private static Arm mArm = Arm.getInstance();
    private static Utils.Vector2D setPoint;
    private static double xMOD;
    private static double yMOD;
    
    public temp_ChangeSetPoint(DoubleSupplier x, DoubleSupplier y) {
        xMOD = (x.getAsDouble()) + 0;
        yMOD = (y.getAsDouble()) + 0;
        
        setPoint = new Utils.Vector2D(xMOD, yMOD);
        addRequirements(mArm);
    }

    @Override
    public void execute() {
        mArm.changeSetpoint(setPoint);
    }
}
