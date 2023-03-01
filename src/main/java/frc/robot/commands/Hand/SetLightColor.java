package frc.robot.commands.Hand;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Leds;

public class SetLightColor extends InstantCommand {
    double mColor;
    public SetLightColor(double color) {
        color = mColor;
        addRequirements(Leds.getInstance());
    }

    public void execute() {
        Leds.getInstance().setColor(mColor);
    }
}
