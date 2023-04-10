package frc.robot.commands.Lights;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Constants.Lights;
import frc.robot.subsystems.Lighting;

public class LightReqCMD extends InstantCommand {
    private final Lighting mLighting = Lighting.getInstance();
    
    Timer timer = Constants.Lights.timer;
    boolean cone;

    /**
     * 90 for Cone, 270 for Cube
     * @param angle POV Angle
     */
    public LightReqCMD() {
        this.addRequirements(mLighting);
        // timer = new Timer(); 
    }
    
    @Override
    public void initialize() {
        timer.restart();
        // Lighting.Cone = cone;
        if (Lighting.Cone) {
            Constants.Lights.PWMVal = Lights.kConeStatic;
            // Lights.lastLight = false;
            mLighting.setLights(Lights.kConeBlink);
            this.setName("Requesting Cone");         
        }
        if (!Lighting.Cone) {
            Constants.Lights.PWMVal = Lights.kCubeStatic;
            // Lights.lastLight = true;            
            mLighting.setLights(Lights.kCubeBlink);
            this.setName("Requesting Cube");
        }
    }

    @Override
    public void execute() {}
}