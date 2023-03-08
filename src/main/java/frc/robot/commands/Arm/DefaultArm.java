package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Utils;
import frc.robot.Utils.Vector2D;
import frc.robot.subsystems.Arm;

public class DefaultArm extends CommandBase {
    private Arm mArm = Arm.getInstance();
    private Vector2D targetPosition, targetPositionAngles;

    public DefaultArm() {
        targetPosition = new Vector2D(0.4, 0.1);
    }

    public DefaultArm(double x, double y) {
        targetPosition = new Vector2D(x, y);
    }

    public void setArmPosition(Vector2D positionInHandCoordinates) {
        targetPositionAngles = mArm.calculateHandPosition(positionInHandCoordinates);
    
        
    }

    public Vector2D getCurrentHandPosition() {
        return mArm.calculateHandPosition(new Utils.Vector2D(mArm.getAbsolutePositionElbow(), mArm.getAbsolutePositionShoulder()));
    }
}
