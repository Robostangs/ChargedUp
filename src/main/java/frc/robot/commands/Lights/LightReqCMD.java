package frc.robot.commands.Lights;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.Lights;
import frc.robot.subsystems.Lighting;

public class LightReqCMD extends InstantCommand {
    private final Lighting mLighting = Lighting.getInstance();
    
    Timer timer = Lighting.timer;
    boolean cone;

    /**
     * Switch between Cone and Cube lights
     */
    public LightReqCMD() {
        this.addRequirements(mLighting);
    }
    
    @Override
    public void initialize() {
        timer.restart();
        if (Lighting.isCone) {
            Lighting.PWMVal = Lights.kConeStatic;
            // Lights.lastLight = false;
            mLighting.setLights(Lights.kConeBlink);
            this.setName("Requesting Cone");         
        }
        if (!Lighting.isCone) {
            Lighting.PWMVal = Lights.kCubeStatic;
            // Lights.lastLight = true;            
            mLighting.setLights(Lights.kCubeBlink);
            this.setName("Requesting Cube");
        }
        Lighting.isCone = !Lighting.isCone;
    }

    @Override
    public void execute() {}
}