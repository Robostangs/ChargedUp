package frc.robot.commands.Lights;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Lights;
import frc.robot.subsystems.Lighting;

public class LightReqCMD extends CommandBase {
    private final Lighting mLighting = Lighting.getInstance();
    Boolean Cone;
    Boolean Cube;
    String reqPiece;
    
    double PWMVal;
    Timer timer;
    double blinkTime = 7.5;
    int angle;

    
    public LightReqCMD(int angle) {
        this.angle = angle; 
        addRequirements(mLighting);
        timer = new Timer(); 
    }
    
    @Override
    public void initialize() {
        timer.restart();
        if (angle == 270) {
            Lighting.lastLight = Lights.kConeStatic;
            mLighting.setLights(Lights.kConeBlink);
            this.setName("Requesting Piece: Cone");
        } if (angle == 90) {
            Lighting.lastLight = Lights.kCubeStatic;
            mLighting.setLights(Lights.kCubeBlink);
            this.setName("Requesting Piece: Cube");
        }
    }

    @Override
    public void execute() {
        if (timer.hasElapsed(blinkTime)) {
            new LightCMD(Lighting.lastLight).schedule();
        }
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Lights set");
    }
}