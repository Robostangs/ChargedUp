package frc.robot.commands.Lights;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Lighting;

public class LightReqCMD extends InstantCommand {
    private final Lighting mLighting = Lighting.getInstance();
    Boolean Cone;
    Boolean Cube;
    String reqPiece;
    
    double coneLight = 0.65;
    double cubeLight = 0.91;

    public LightReqCMD(int angle) {
        addRequirements(mLighting);
        if (angle == 270) {
            Cone = true;
            Cube = false;
        } if (angle == 90) {
            Cone = false;
            Cube = true;
        }
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (Cone) {
            mLighting.setLights(coneLight);
            reqPiece = "Cone";
        } else if (Cube) {
            mLighting.setLights(cubeLight);
            reqPiece = "Cube";
        } else {
            mLighting.setLights(mLighting.killLights());
            reqPiece = "None";
        }
    }

    public void end() {
        SmartDashboard.putString("Requested piece", reqPiece);
    }
}