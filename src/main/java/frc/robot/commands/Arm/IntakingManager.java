package frc.robot.commands.Arm;

import java.io.Console;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Vision;
import frc.robot.Utils.Vector2D;
import frc.robot.Utils.Vector3D;
import frc.robot.autos.basicTranslate;
import frc.robot.autos.exampleAuto;
import frc.robot.autos.rotation;
import frc.robot.commands.Hand.SetGrip;
import frc.robot.commands.Hand.ToggleGrip;
import frc.robot.commands.Hand.ToggleHolding;
import frc.robot.commands.Swerve.Flatten;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Hand;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Arm.ArmPosition;

public class IntakingManager extends CommandBase {
    private static final Arm mArm = Arm.getInstance();
    private static final Swerve mDrivetrain = Swerve.getInstance();
    private static final Vision mVision = Vision.getInstance();
    private static final Hand mHand = Hand.getInstance();

    private double mTX, mTY;
    private Vector2D mVector2d;
    private double mTargetX, mTargetY, mFinalX, mFinalY;

    public IntakingManager() {
        setName("intaking Manager");
        addRequirements(mDrivetrain, mArm, mHand);
        // try {         

        //     addCommands(
        //         new InstantCommand(() ->             DataLogManager.log(
        //             "Intaking Manager info" + 
        //             "Limelight TX: " + mVector2d.x + "," +
        //             "Limelight TY: " + mVector2d.y + "," +
        //             "Code TX: " + mTX + "," +
        //             "Code TY: " + mTY + "," +
        //             "Theta 1: " + mArm.getShoulderPositionFromMotor() + "," +
        //             "Target X: " + mTargetX + "," +
        //             "Target Y: " + mTargetY + "," 
        //         )),

        //         new ParallelDeadlineGroup(
        //             new basicTranslate(mDrivetrain, new Vector3D(((sin(90-mTX) * Constants.Arm.upperarmLength) / sin(90+mTX-mArm.getShoulderPositionFromMotor())) - (0.84)
        //                                                             , (((sin(90-mTX) * Constants.Arm.upperarmLength) / sin(90+mTX-mArm.getShoulderPositionFromMotor())) - (0.84)) * sin(mTX), 0)), 
        //             new SetGrip()).withTimeout(2.5)
        //         );
        // } catch (Exception e) {
            
        // }
    }

    @Override
    public void initialize() {
        mVector2d = mVision.objectPosition();
            
        mTX = -mVector2d.x;
        mTY = -mVector2d.y;

        mTargetX = Math.abs(((sin(90-mTY) * Constants.Arm.upperarmLength) / sin(90+mTY-mArm.getShoulderPositionFromMotor())))- (mArm.getHandPositionX() -0.2) + 0.05;
        mTargetY = (mTargetX * tan(mTX)) - 0.11 -0.2;

        mFinalX = cos(mDrivetrain.getRawGyroAngle() - Math.toDegrees(Math.atan2(mTargetY, mTargetX))) * Math.sqrt(Math.pow(mTargetX, 2) + Math.pow(mTargetY, 2));
        mFinalY = (sin(mDrivetrain.getRawGyroAngle() - Math.toDegrees(Math.atan2(mTargetY, mTargetX))) * Math.sqrt(Math.pow(mTargetX, 2) + Math.pow(mTargetY, 2)));

        

        DataLogManager.log(
                        "Intaking Manager info" + 
                        "Limelight TX: " + mVector2d.x + ",\n" +
                        "Limelight TY: " + mVector2d.y + ",\n" +
                        "Code TX: " + mTX + ",\n" +
                        "Code TY: " + mTY + ",\n" +
                        "Theta 1: " + mArm.getShoulderPositionFromMotor() + ",\n" +
                        "Target X: " + mTargetX + ",\n" +
                        "Target Y: " + mTargetY + ",\n" +            
                        "Final X: " + mFinalX + ",\n" +
                        "Final Y: " + mFinalY + ",\n" 
                    );

        SmartDashboard.putString("Intaking Manager Info", 
            "Intaking Manager info" + 
            "Limelight TX: " + mVector2d.x + "," +
            "Limelight TY: " + mVector2d.y + "," +
            "Code TX: " + mTX + "," +
            "Code TY: " + mTY + "," +
            "Theta 1: " + mArm.getShoulderPositionFromMotor() + "," +
            "Target X: " + mTargetX + "," +
            "Target Y: " + mTargetY + "," +            
            "Target X: " + mFinalX + "," +
            "Target Y: " + mFinalY + "," 
        );
    
        new SetGrip().withTimeout(2.5).schedule();
        new basicTranslate(mDrivetrain, new Vector3D(-mFinalX, -mFinalY, 0)).schedule();
    }
    


    public double sin(double value) {
        return Math.sin(Math.toRadians(value));
    }

    public double cos(double value) {
        return Math.cos(Math.toRadians(value));
    }

    public double tan(double value) {
        return Math.tan(Math.toRadians(value));
    }
    
}
