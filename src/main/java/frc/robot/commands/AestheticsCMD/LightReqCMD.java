package frc.robot.commands.AestheticsCMD;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Aesthetics.Lighting;

import java.sql.Time;
import java.time.Duration;

public class LightReqCMD extends InstantCommand {
    private final Lighting mLighting = Lighting.getInstance();
    Boolean Cone;
    Boolean Cube;
    String reqPiece;
    double coneLight = 0.65;
    double cubeLight = 0.91;
    private int angle;
    public static int lastAngle;
    public LightReqCMD(int Angle) {
        angle = Angle;
        addRequirements(Lighting.getInstance());
        if (Angle == 270) {
            Cone = true;
            Cube = false;
        } if (Angle == 90) {
            Cone = false;
            Cube = true;
        } if (Angle == 180) {
            Cone = false;
            Cube = false;
        }
    }

@Override
public void initialize() {
    // TODO Auto-generated method stub
    lastAngle=angle;
}
    @Override
    public void execute() {
        if (Cone) {
            mLighting.setLights(coneLight);
            reqPiece = "Cone";
        } else if (Cube) {
            mLighting.setLights(cubeLight);
            reqPiece = "Cube";
        } else {
            mLighting.killLights();
            reqPiece = "None";
        }
    }

    // @Override
    // public void end(boolean interrupted) {
    //     mLighting.killLights();
    //     if (Cone) {
    //         mLighting.setLights(coneLight);
    //         reqPiece = "Cone";
    //     } else if (Cube) {
    //         mLighting.setLights(cubeLight);
    //         reqPiece = "Cube";
    //     }
    //     SmartDashboard.putString("Requested piece", reqPiece);
    //     // mLighting.killLights();
    // }
}