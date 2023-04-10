package frc.robot.autos;

import java.util.ArrayList;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

                ArrayList<Translation2d> points = new ArrayList<Translation2d>();

                if(SmartDashboard.getBoolean("Open Side?", true)) {
                        points.add(new Translation2d(1.91, 3.82)); // Starting the Trajectory Position
                        points.add(new Translation2d(6, 4.4));  // Final Position before starting Auto Grab
                        points.add(new Translation2d(2.3, 4.82));  // Midpoint to avoid hitting the charge station
                        points.add(new Translation2d(6.6, 4.9));   // Starting the Second Path a little bit to the left to the cube
                        points.add(new Translation2d(1.87, 4.45));// Finish point infront of cube placement
                } else {
                        points.add(new Translation2d(1.878, 1.81)); // Starting the Trajectory Position
                        points.add(new Translation2d(6.111, 0.859));  // Final Position before starting Auto Grab
                        points.add(new Translation2d(2.3, 0.709));  // Midpoint to avoid hitting the charge station
                        points.add(new Translation2d(6.6, 0.494));   // Starting the Second Path a little bit to the right to the cube
                        points.add(new Translation2d(1.436, 1.11));// Finish point infront of cube placement
                }

                // pretending i am working so gary doesnt bother me

                // pretending i am working so gary doesnt bother me
                // pretending i am working so gary doesnt bother me
                // pretending i am working so gary doesnt bother me
                // pretending i am working so gary and nandan dont bother me
                // pretending i am working so gary and nandan dont bother me





                // An example trajectory to follow. All units in meters.

                        // if(SmartDashboard.getBoolean("isRed", false)) {
                        // s_Swerve.setGyro(0);
                        // s_Swerve.updateOdometryManual(1.91,
                        //                 3.82,
                        //                 Rotation2d.fromRadians(-3.1336563122823273).getDegrees()
                        //                 );
                        // } else {
                        // s_Swerve.setGyro(180);
                        // s_Swerve.updateOdometryManual(exampleTrajectory.getInitialPose().getX(),
                        // exampleTrajectory.getInitialPose().getY(), 0);
                        // }

                        PathPlannerState unFixedInitialState = new PathPlannerState();
                        unFixedInitialState.poseMeters = new Pose2d(points.get(0),Rotation2d.fromDegrees(0));
                        unFixedInitialState.holonomicRotation = Rotation2d.fromDegrees(180);
                        PathPlannerState initialState = PathPlannerTrajectory.horseyWpiBlueTransformStateForAlliance(
                                unFixedInitialState, 
                                DriverStation.getAlliance());

                        


                        //Maybe make this first x smaller
                        Command firstPath =  translatePp.getMicroManageTranslateCommand(
                                () -> new PathPoint(
                                        points.get(0), 
                                        Rotation2d.fromDegrees(0), 
                                        Rotation2d.fromDegrees(180)),
                                ()->new PathPoint(points.get(1),
                                                  Rotation2d.fromDegrees(0), 
                                                  Rotation2d.fromDegrees(0)),
                                () -> new PathPoint(points.get(2),
                                                  Rotation2d.fromDegrees(0), 
                                                  Rotation2d.fromDegrees(Swerve.getInstance().getPose().getRotation().getDegrees()-185)),
                                ()->Rotation2d.fromDegrees(Swerve.getInstance().getPose().getRotation().getDegrees()-180));

                        Command secondPath = translatePp.getCompleteTranslateCommand(
                                () -> new PathPoint(points.get(3), 
                                                    Rotation2d.fromDegrees(0), 
                                                    Rotation2d.fromDegrees(0)), 
                                () -> new PathPoint(points.get(4), 
                                                    Rotation2d.fromDegrees(0), 
                                                    Rotation2d.fromDegrees(180)));

                        // Command secondPath = translatePp.getAbsoluteTranslateCommand(()->new PathPoint(
                        //         new Translation2d(1.536, 4.05),
                        //         Rotation2d.fromDegrees(0),
                        //         Rotation2d.fromDegrees(180)));



                        addCommands(
                                new InstantCommand(()-> s_Swerve.updateOdometryManual(initialState.poseMeters.getX(),
                                initialState.poseMeters.getY(),
                                initialState.holonomicRotation.getDegrees())),
                                new InstantCommand(()->SmartDashboard.putString("TripleAuto", "High Arm")),
                                ProfiledChangeSetPoint.createWithTimeout(() -> Constants.Arm.SetPoint.coneHighPositionBad),
                                new InstantCommand(()->SmartDashboard.putString("TripleAuto", "Waiting")),
                                new WaitCommand(0.2),
                                new InstantCommand(()->SmartDashboard.putString("TripleAuto", "Reset Lash"))
                                ,
                                //  new InstantCommand(() -> s_Arm.resetLash()),
                                new InstantCommand(()->SmartDashboard.putString("TripleAuto", "Set Grip")),

                                new SetGrip().withTimeout(0.7),
                                new InstantCommand(()->SmartDashboard.putString("TripleAuto", "General Intake And SetGrip for 2.5s and first path")),

                                new ParallelCommandGroup(
                                        ProfiledChangeSetPoint.createWithTimeout(
                                                                        () -> Constants.Arm.SetPoint.generalIntakePosition),
                                        new SetGrip().withTimeout(2.5),
                                        firstPath
                                ),
                                // new InstantCommand(() -> s_Swerve.resetOdometry(new )),
                                new InstantCommand(()->SmartDashboard.putString("TripleAuto", "Auto Grab")),

                                charlieAutoGrab.getCommand().withTimeout(5).andThen(new LoggyPrintCommand("exited")),
                                        //new LoggyPrintCommand("exited2"),
                                new InstantCommand(()->SmartDashboard.putString("TripleAuto", "Second Path along with High Arm")),

                                new ParallelCommandGroup(secondPath, new SequentialCommandGroup(new WaitCommand(1.5), ProfiledChangeSetPoint.createWithTimeout(() -> Constants.Arm.SetPoint.cubeHighPosition))),
                                new InstantCommand(()->SmartDashboard.putString("TripleAuto", "Wait")),
                                new InstantCommand(()->SmartDashboard.putString("TripleAuto", "Open Grip for 0.7s")),

                                new SetGrip().withTimeout(0.7),
                                new InstantCommand(()->SmartDashboard.putString("TripleAuto", "Stow and open grip for 2.5s")),

                                new ParallelDeadlineGroup(
                                        ProfiledChangeSetPoint.createWithTimeout(
                                                        () -> Constants.Arm.SetPoint.stowPosition),
                                        new SetGrip()).withTimeout(2.5));
        }
}
