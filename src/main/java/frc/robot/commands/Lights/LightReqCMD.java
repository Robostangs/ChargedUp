package frc.robot.commands.Lights;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.Lights;
import frc.robot.subsystems.Lighting;

public class LightReqCMD extends CommandBase {
    private final Lighting mLighting = Lighting.getInstance();
    Boolean Cone;
    Boolean Cube;
    String reqPiece;
    
    double PWMVal;
    Timer timer;
    double blinkTime = 3; // Revert to 7.5
    Constants.Lights.ConeCube object;

    /**
     * Request lights for Cone and Cube
     * @param var Use enum {@link Constants.Lights.ConeCube}
     */
    public LightReqCMD() {
        addRequirements(mLighting);
        timer = new Timer(); 
    }
    
    @Override
    public void initialize() {
        Constants.Lights.prevLightReqCMD = this;

        timer.restart();

        if (Lighting.lastLight == Constants.Lights.ConeCube.kCone) {
            // Do Cube Lighting
            Lighting.lastLight = Constants.Lights.ConeCube.kCube;
            PWMVal = Constants.Lights.kCubeStatic;
            mLighting.setLights(Lights.kCubeBlink);
            this.setName("Requesting Piece: Cube");
        } else if (Lighting.lastLight == Constants.Lights.ConeCube.kCube) {
            // Do Cone Lighting
            Lighting.lastLight = Constants.Lights.ConeCube.kCone;
            PWMVal = Constants.Lights.kConeStatic;
            mLighting.setLights(Lights.kConeBlink);
            this.setName("Requesting Piece: Cone");
        } else {
            System.out.println(Lighting.lastLight.toString());
        }
        // switch(object) {
        //     case kCone:
        //         Lighting.lastLight = Constants.Lights.ConeCube.kCone;
        //         PWMVal = Constants.Lights.kConeStatic;
        //         mLighting.setLights(Lights.kConeBlink);
        //         this.setName("Requesting Piece: Cone");
        //         break;
        //     case kCube:
        //         Lighting.lastLight = Constants.Lights.ConeCube.kCube;
        //         PWMVal = Constants.Lights.kCubeStatic;
        //         mLighting.setLights(Lights.kCubeBlink);
        //         this.setName("Requesting Piece: Cube");
        //         break;
        // }
    }

    @Override
    public void execute() {
        if (timer.hasElapsed(blinkTime)) {
            System.out.println(this.getName());
            new LightCMD(PWMVal).schedule();
        }
    }

    @Override
    public void end(boolean interrupted) {
        Constants.Lights.prevLightReqCMD = null;

    }
}