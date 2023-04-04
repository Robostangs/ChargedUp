package frc.robot.autos;

import frc.robot.Utils;
import frc.robot.Utils.Vector2D;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

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