package frc.robot.autos;

import frc.LoggyThings.LoggyPrintCommand;
import frc.robot.Constants;
import frc.robot.Utils;
import frc.robot.Utils.Vector2D;
import frc.robot.subsystems.Swerve;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class translateWithoutTrajectory extends CommandBase {
    private Swerve s_Swerve;
    private Vector2D position;
    public translateWithoutTrajectory(Utils.Vector2D position){
        addRequirements(Swerve.getInstance());
        s_Swerve = Swerve.getInstance();
        this.position = position;
        
    }

    @Override
    public void initialize() {
        s_Swerve.drive(new Translation2d(position.x, position.y), 0, false, false);
    }
}