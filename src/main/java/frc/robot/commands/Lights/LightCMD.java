package frc.robot.commands.Lights;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Lighting;;

public class LightCMD extends InstantCommand {
    private final Lighting mLighting = Lighting.getInstance();

    private double PWMVal;

    public LightCMD(double color) {
        PWMVal = color;
        this.addRequirements(mLighting);
    }

    @Override
    public void initialize() {
    //     if (PWMVal == Constants.Lights.kConeBlink || PWMVal == Constants.Lights.kConeStatic) {
    //         Lighting.Cone = false;
    //     } else {
    //         Lighting.Cone = true;
    //     }
    }
    
    @Override
    public void execute() {
        mLighting.setLights(PWMVal);
    }
}