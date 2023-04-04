package frc.robot.autos;

import java.util.function.Supplier;

import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Utils.Vector2D;
import frc.robot.Vision.LimelightState;
import frc.robot.Vision;
import frc.robot.subsystems.Swerve;

public class charlieAutoDriveToCube{
    
    public static Command getCommand() {
        return translatePp.getTheThing(getPathPoint());
    }


    public static Supplier<PathPoint> getPathPoint() {
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
            return ()-> new PathPoint(fieldSpacePieceDistT2d, finalAngle, finalAngle);
    }

}
