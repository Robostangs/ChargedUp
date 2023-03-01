package frc.robot.commands.AestheticsCMD;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Aesthetics.Music;

public class WarningBeep extends InstantCommand {
    private final Music mMusic = Music.getInstance();
    private final WaitCommand wait = new WaitCommand(0.5);

    public WarningBeep() {}

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        for (int x = 0; x < 3; x++) {
            mMusic.warningSound(110);
            wait.schedule();
            mMusic.warningSound(0);
            wait.schedule();
        }
    }

    public void end() {
    }
}

