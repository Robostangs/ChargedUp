package frc.robot.autos;

import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.LoggyThings.LoggyPrintCommand;
import frc.robot.Constants;
import frc.robot.Utils.Vector2D;
import frc.robot.Vision.LimelightState;
import frc.robot.Vision;
import frc.robot.commands.Arm.ProfiledChangeSetPoint;
import frc.robot.commands.Hand.SetGrip;
import frc.robot.subsystems.Swerve;

public class charlieAutoGrab{
    public static SequentialCommandGroup getCommand() {
        // if (!Vision.getInstance().targetVisible(LimelightState.center)) {
        //     DataLogManager.log("NO THING TO GRABBBB!!!!");
        //     return null;
        // } else {

        new ConditionalCommand(
            new SequentialCommandGroup(new InstantCommand(() -> LoggyPrintCommand("it no work"))),  
            new SequentialCommandGroup(
            ProfiledChangeSetPoint.createWithTimeout(() -> Constants.Arm.SetPoint.generalIntakePosition).alongWith(new LoggyPrintCommand("First Part")),
            // new WaitCommand(1),
            relativeTranslate.getCommand(()->getRelativeFieldSpaceCubePoint()).deadlineWith(new SetGrip()).alongWith(new LoggyPrintCommand("SecondPart")),
            ProfiledChangeSetPoint.createWithTimeout(() -> Constants.Arm.SetPoint.stowPosition).alongWith(new LoggyPrintCommand("Third Part"))), 
            !Vision.getInstance().targetVisible(LimelightState.center));
       
    }

    public static PathPoint getRelativeFieldSpaceCubePoint() {
            Vector2D robotSpacePiecePos = Vision.getInstance().calculateAndPrintGamePiecePosition();
            Vector2D fieldSpacePieceDist = robotSpacePiecePos
                    .getRotatedBy(Swerve.getInstance().getPose().getRotation().getRadians());
            // Vector2D mVector2d = Vision.getInstance().objectPosition();

            SmartDashboard.putNumber("Auto Grab/Target/X", robotSpacePiecePos.x);
            SmartDashboard.putNumber("Auto Grab/Target/Y", robotSpacePiecePos.y);
            SmartDashboard.putNumber("Auto Grab/Final/X", fieldSpacePieceDist.x);
            SmartDashboard.putNumber("Auto Grab/Final/Y", fieldSpacePieceDist.y);

            Translation2d fieldSpacePieceDistT2d = fieldSpacePieceDist.toTranslation2d();
            Rotation2d finalAngle = fieldSpacePieceDistT2d.getAngle();
            return new PathPoint(fieldSpacePieceDistT2d, finalAngle, finalAngle);
    }
}

