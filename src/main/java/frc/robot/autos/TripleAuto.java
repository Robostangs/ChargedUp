package frc.robot.autos;

import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Hand;
import frc.robot.subsystems.Swerve;

public class TripleAuto extends SequentialCommandGroup {
        public TripleAuto() {
                Swerve s_Swerve = Swerve.getInstance();
                Arm s_Arm = Arm.getInstance();
                Hand s_Hand = Hand.getInstance();
                addRequirements(s_Swerve, s_Arm, s_Hand);
                setName("Triple Auto");

                // An example trajectory to follow. All units in meters.

                        // if(SmartDashboard.getBoolean("isRed", false)) {
                        // s_Swerve.setGyro(0);
                        s_Swerve.updateOdometryManual(1.91,
                                        3.82,
                                        Rotation2d.fromRadians(-3.1336563122823273).getDegrees()
                                        );
                        // } else {
                        // s_Swerve.setGyro(180);
                        // s_Swerve.updateOdometryManual(exampleTrajectory.getInitialPose().getX(),
                        // exampleTrajectory.getInitialPose().getY(), 0);
                        // }


                        //Maybe make this first x smaller
                        Command firstPath =  translatePp.getMicroManageTranslateCommand(()->new PathPoint(new Translation2d(5.51, 4.4),
                        Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
                        () -> new PathPoint(new Translation2d(2.3, 4.82), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(Math.abs(Swerve.getInstance().getPose()
                        .getRotation().getDegrees() - 45))),
                        ()->Rotation2d.fromDegrees(Math.abs(Swerve.getInstance().getPose()
                        .getRotation().getDegrees()-180)));

                        Command secondPath = translatePp.getCompleteTranslateCommand(
                                () -> new PathPoint(new Translation2d(6.6, 4.9) , Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)), 
                                () -> new PathPoint(new Translation2d(1.436, 4.05), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(180)));
                        // Command secondPath = translatePp.getAbsoluteTranslateCommand(()->new PathPoint(
                        //         new Translation2d(1.536, 4.05),
                        //         Rotation2d.fromDegrees(0),
                        //         Rotation2d.fromDegrees(180)));

                        addCommands(
                                ProfiledChangeSetPoint.createWithTimeout(() -> Constants.Arm.SetPoint.coneHighPositionBad),
                                new WaitCommand(0.2), new InstantCommand(() -> s_Arm.resetLash()),
                                new SetGrip().withTimeout(0.7),

                                new ParallelCommandGroup(
                                        ProfiledChangeSetPoint.createWithTimeout(
                                                                        () -> Constants.Arm.SetPoint.generalIntakePosition),
                                        new SetGrip().withTimeout(2.5),
                                        firstPath
                                ),
                                // new InstantCommand(() -> s_Swerve.resetOdometry(new )),

                                charlieAutoGrab.getCommand().withTimeout(5).andThen(new LoggyPrintCommand("exited")),
                                        // new LoggyPrintCommand("exited2"),
                                
                                new ParallelCommandGroup(secondPath, new SequentialCommandGroup(new WaitCommand(1.8), ProfiledChangeSetPoint.createWithTimeout(() -> Constants.Arm.SetPoint.cubeHighPosition))),
                                new WaitCommand(0.5),
                                new SetGrip().withTimeout(0.7),
                                new ParallelDeadlineGroup(
                                        ProfiledChangeSetPoint.createWithTimeout(
                                                        () -> Constants.Arm.SetPoint.stowPosition),
                                        new SetGrip()).withTimeout(2.5));
        }
}
