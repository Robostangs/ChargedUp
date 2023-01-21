package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Vision;
import frc.robot.Utils.Vector3D;
import frc.robot.Vision.LimelightState;
import frc.robot.subsystems.Drivetrain;

public class GetToPosition extends CommandBase {
    Drivetrain mDrivetrain = Drivetrain.getInstance();
    Vision mVision = Vision.getInstance();
    double mSpeed;
    Vision.LimelightState mState;
    private Vector3D v1, v2;

    public GetToPosition(double speed, Vision.LimelightState state) {
        addRequirements(mDrivetrain);
        setName("Straighten Against Wall");
        mSpeed = speed;
        mState = state;
    }

    @Override
    public void execute() {
        if(mVision.targetVisible(LimelightState.leftLimelight)) {
            v1 = mVision.getLimelightPosition(LimelightState.leftLimelight);

            if(v1.y + Constants.Drivetrain.targetOffset < mDrivetrain.getPose().getY()) {
                mDrivetrain.drive(new ChassisSpeeds(0, mSpeed, 0));
            } else if(v1.y + Constants.Drivetrain.targetOffset > mDrivetrain.getPose().getY()) {
                mDrivetrain.drive(new ChassisSpeeds(0, -mSpeed, 0));
            }
        } else if(mVision.targetVisible(LimelightState.rightLimelight)) {
            v2 = mVision.getLimelightPosition(LimelightState.rightLimelight);
            
            if(v2.y - Constants.Drivetrain.targetOffset > mDrivetrain.getPose().getY()) {
                mDrivetrain.drive(new ChassisSpeeds(0, mSpeed, 0));
            } else if(v1.y - Constants.Drivetrain.targetOffset < mDrivetrain.getPose().getY()) {
                mDrivetrain.drive(new ChassisSpeeds(0, -mSpeed, 0));
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        mDrivetrain.drive(new ChassisSpeeds(0, 0, 0));
    }

}
