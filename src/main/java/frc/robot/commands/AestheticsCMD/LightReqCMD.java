package frc.robot.commands.AestheticsCMD;

import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Aesthetics.Lighting;

public class LightReqCMD extends CommandBase {
    private final Lighting mLighting = Lighting.getInstance();
    Boolean Cone;
    Boolean Cube;
    
    double coneLight = 0;
    double cubeLight = -0.57;

    public LightReqCMD(int Angle) {
        if (Angle == 90) {
            Cone = true;
            Cube = false;
        } if (Angle == 270) {
            Cone = false;
            Cube = true;
        } if (Angle == 180) {
            Cone = false;
            Cube = false;
        }
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (Cone) {
            mLighting.setLights(coneLight);
        } else if (Cube) {
            mLighting.setLights(cubeLight);
        } else {
            mLighting.setLights(0);
        }
    }

    public void end() {
        SmartDashboard.putString("Music Player", "Deactivated");
    }
}