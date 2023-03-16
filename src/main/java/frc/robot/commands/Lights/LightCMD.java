package frc.robot.commands.Lights;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Lighting;;

public class LightCMD extends InstantCommand {
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