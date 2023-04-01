package frc.robot.commands.Lights;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Lighting;

public class LightReqCMD extends SequentialCommandGroup {
    private final Lighting mLighting = Lighting.getInstance();
    Boolean Cone;
    Boolean Cube;
    String reqPiece;
    
    double PWMVal;
    Timer timer;

    public LightReqCMD(int angle) {
        if (angle == 270) {
            // Cone = true;
            // Cube = false;
            PWMVal = mLighting.coneLight;
            this.setName("Requesting Piece: Cone");
        } if (angle == 90) {
            // Cone = false;
            // Cube = true;
            PWMVal = mLighting.cubeLight;
            this.setName("Requesting Piece: Cube");
        }
        
        this.addRequirements(mLighting);
        timer = new Timer();
        this.addCommands(
            new RepeatCommand(
                new LightCMD(PWMVal).andThen(
                    new WaitCommand(0.5)).andThen(
                        new LightCMD(mLighting.killLights).andThen(
                            new WaitCommand(0.5)
                        )
                    )
            ).until(() -> timer.get() >= 10)
        );

        // Another Idea:
        /*
        this.addCommands(
            new RepeatCommand(
                new LightCMD(PWMVal).andThen(
                    new WaitCommand(0.5)).andThen(
                        new LightCMD(mLighting.killLights).andThen(
                            new WaitCommand(0.5)
                        )
                    )
            ).alongWith(new WaitCommand(10))
        );
        */
    }

    // @Override
    // public void execute() {
    //     if (Cone) {
    //         mLighting.setLights(coneLight);
    //         reqPiece = "Cone";
    //     } else if (Cube) {
    //         mLighting.setLights(cubeLight);
    //         reqPiece = "Cube";
    //     } else {
    //         mLighting.killLights();
    //         reqPiece = "None";
    //     }
    // }

    public void end() {
        SmartDashboard.putString("Requested piece", reqPiece);
        mLighting.setLights(PWMVal);
    }
}