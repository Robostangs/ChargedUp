package frc.robot.autos;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.commands.Arm.ProfiledChangeSetPoint;
import frc.robot.subsystems.Swerve;

public class pathPlannerChooser {
    private final Swerve mSwerve = Swerve.getInstance();
    public boolean autonFinished;
    private PathPlannerTrajectory pathPlannerTrajectory;
    private String selectedAuton = "";
    private FieldObject2d pathTrajectory;
    private double posesPerTrajectory;
    private List<Pose2d> pathPoses = new ArrayList<>();

    private HashMap<String, Command> eventMap1 = new HashMap<String, Command>();

    /** Do not include file extensions: .exe, .json, .path */
    public pathPlannerChooser(String path) {
        autonFinished = false;
        this.selectedAuton = path;
        if (!selectedAuton.equals("null")) {
            try {
                pathPlannerTrajectory = PathPlanner.loadPath(
                        path,
                        PathPlanner.getConstraintsFromPath(path));
                pathPlannerTrajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(pathPlannerTrajectory,
                        DriverStation.getAlliance());
            } catch (Exception e) {
                /* Blank Trajectory that will just chill */
                pathPlannerTrajectory = PathPlanner.generatePath(
                        new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared),
                        new PathPoint(mSwerve.getPose().getTranslation(),
                                mSwerve.getPose().getRotation(),
                                mSwerve.getPose().getRotation()),
                        new PathPoint(mSwerve.getPose().getTranslation(),
                                mSwerve.getPose().getRotation(),
                                mSwerve.getPose().getRotation()));
                System.out.println("PathPlannerChooser: " + e);
            }

            mSwerve.resetOdometry(pathPlannerTrajectory.getInitialHolonomicPose());
            autonPoses();
        }
        eventMap1.put("done", new InstantCommand(() -> autonFinished = true));
        eventMap1.put("start", Commands.print("Start"));
        eventMap1.put("print", Commands.print("print"));
        eventMap1.put("testmark",
                ProfiledChangeSetPoint.createWithTimeout(() -> Constants.Arm.SetPoint.generalIntakePosition));
        // eventMap1.put("balance", new balance());
    }

    public Command generateTrajectory() {
        if (selectedAuton.equals("null")) {
            return Commands.print("Null Path");
        } else {
            return new FollowPathWithEvents(
                    new PPSwerveControllerCommand(
                            pathPlannerTrajectory,
                            mSwerve::getPose,
                            Constants.Swerve.swerveKinematics,
                            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                            new PIDController(Constants.AutoConstants.kPThetaController, 0, 0),
                            mSwerve::setModuleStates,
                            mSwerve),
                    pathPlannerTrajectory.getMarkers(),
                    eventMap1);

            // new PPSwerveControllerCommand(
            // pathPlannerTrajectory,
            // mSwerve::getPose,
            // new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            // new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            // new PIDController(Constants.AutoConstants.kPThetaController, 0, 0),
            // null,
            // mSwerve),
            // pathPlannerTrajectory.getMarkers(),
            // eventMap1);
        }
    }

    public Trajectory getTrajectory() {
        return pathPlannerTrajectory;
    }

    private void autonPoses() {
        pathTrajectory = mSwerve.getField().getObject(Constants.AutoConstants.kFieldObjectName);
        posesPerTrajectory = Math.floor(pathPlannerTrajectory.getStates().size() / 85);
        // System.out.println(pathPlannerTrajectory.getStates().size());
        // System.out.println(posesPerTrajectory);
        for (int x = 0; x < pathPlannerTrajectory.getStates().size(); x++) {
            if (x % posesPerTrajectory == 0) {
                // System.out.println(x);
                pathPoses.add(pathPlannerTrajectory.getStates().get(x).poseMeters);
            }
        }

        pathPoses.add(pathPlannerTrajectory.getEndState().poseMeters);
        pathTrajectory.setPoses(pathPoses);
        SmartDashboard.putString("Path Trajectory", pathPoses.toArray().toString());
    }

    public void closeObject() {
        if (pathTrajectory != null) {
            pathTrajectory.close();
        } else {
            System.out.println("Path Trajectory is null");
        }
    }
}
