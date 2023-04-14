package frc.robot.autos;

import java.util.ArrayList;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.LoggyThings.LoggyPrintCommand;
import frc.robot.Constants;
import frc.robot.commands.Arm.ProfiledChangeSetPoint;
import frc.robot.commands.Hand.SetGrip;
import frc.robot.commands.Swerve.balance;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Hand;
import frc.robot.subsystems.Swerve;

public class upandover extends SequentialCommandGroup {
        public upandover() {
                Swerve s_Swerve = Swerve.getInstance();
                Arm s_Arm = Arm.getInstance();
                Hand s_Hand = Hand.getInstance();
                addRequirements(s_Swerve, s_Arm, s_Hand);
                setName("Up And Over");

                ArrayList<Translation2d> points = new ArrayList<Translation2d>();


                        PathPlannerState unFixedInitialState = new PathPlannerState();
                        unFixedInitialState.poseMeters = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
                        unFixedInitialState.holonomicRotation = Rotation2d.fromDegrees(180);
                        PathPlannerState initialState = PathPlannerTrajectory.horseyWpiBlueTransformStateForAlliance(
                                unFixedInitialState, 
                                DriverStation.getAlliance());

                        


                        //Maybe make this first x smaller
                        Command firstPath = translatePp.getRelativeTranslateCommandWithVelo(() -> new PathPoint(new Translation2d(5, 0), Swerve.getInstance().getPose().getRotation(), Swerve.getInstance().getPose().getRotation()), 1.5);
                        Command secondPath = translatePp.getRelativeTranslateCommandWithVelo(() -> new PathPoint(new Translation2d(-2, 0), Swerve.getInstance().getPose().getRotation().plus(Rotation2d.fromDegrees(180)), Swerve.getInstance().getPose().getRotation()), 1.5);

                        addCommands(
                                // new InstantCommand(()-> s_Swerve.updateOdometryManual(initialState.poseMeters.getX(),
                                // initialState.poseMeters.getY(),
                                // initialState.holonomicRotation.getDegrees())),
                                new InstantCommand(()->SmartDashboard.putString("upandover", "High Arm")),
                                ProfiledChangeSetPoint.createWithTimeout(() -> Constants.Arm.SetPoint.coneHighPositionBad),
                                new InstantCommand(()->SmartDashboard.putString("upandover", "Waiting")),
                                new WaitCommand(0.2),
                                new InstantCommand(()->SmartDashboard.putString("upandover", "Reset Lash"))                                ,
                                //  new InstantCommand(() -> s_Arm.resetLash()),
                                new InstantCommand(()->SmartDashboard.putString("upandover", "Set Grip")),

                                new SetGrip().withTimeout(0.7),
                                new InstantCommand(()->SmartDashboard.putString("upandover", "General Intake And SetGrip for 2.5s and first path")),

                                new ParallelCommandGroup(
                                        ProfiledChangeSetPoint.createWithTimeout(
                                                                        () -> Constants.Arm.SetPoint.stowPosition),
                                        firstPath
                                ),

                                new InstantCommand(()->SmartDashboard.putString("upandover", "Stow and open grip for 2.5s")),
                                secondPath,

                                new balance()
                               //  secondPath
                        );
        }
}
