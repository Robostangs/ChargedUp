package frc.robot.commands.AestheticsCMD;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Aesthetics.Lighting;

import java.sql.Time;
import java.time.Duration;

public class LightReqCMD extends CommandBase {
    private final Lighting mLighting = Lighting.getInstance();
    Boolean Cone;
    Boolean Cube;
    String reqPiece;
    Timer timer  = new Timer();    
    double coneLight = 0.65;
    double cubeLight = 0.91;

    public LightReqCMD(int Angle) {
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
        timer.restart();
    }

    @Override
    public void execute() {
        if (Cone&&timer.get()*5%1>0.5) {
            mLighting.setLights(coneLight);
            reqPiece = "Cone";
        } else if (Cube&&timer.get()*5%1>0.5) {
            mLighting.setLights(cubeLight);
            reqPiece = "Cube";
        } else {
            mLighting.killLights();
            reqPiece = "None";
        }
    }
    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return (timer.get()>5);
    }
    @Override
    public void end(boolean interrupted) {
        mLighting.killLights();
        if (Cone) {
            mLighting.setLights(coneLight);
            reqPiece = "Cone";
        } else if (Cube) {
            mLighting.setLights(cubeLight);
            reqPiece = "Cube";
        }
        SmartDashboard.putString("Requested piece", reqPiece);
        // mLighting.killLights();
    }
}