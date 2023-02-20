package frc.robot.commands.Aesthetics;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lighting;

public class LightCMD extends CommandBase {
    private final Lighting mLighting = Lighting.getInstance();

    public LightCMD() {}

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        mLighting.setLights(-0.57);
    }

    public void end() {
        SmartDashboard.putString("Music Player", "Deactivated");
    }
}