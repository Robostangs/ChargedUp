package frc.robot.commands.AestheticsCMD;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Aesthetics.Lighting;;

public class LightCMD extends CommandBase {
    private final Lighting mLighting = Lighting.getInstance();

    private double PWMVal;

    public LightCMD(double color) {
        PWMVal = color;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        mLighting.setLights(PWMVal);
    }

    public void end() {
        SmartDashboard.putString("Music Player", "Deactivated");
    }
}