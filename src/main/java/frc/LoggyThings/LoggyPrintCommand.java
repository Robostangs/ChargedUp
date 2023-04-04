package frc.LoggyThings;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class LoggyPrintCommand extends InstantCommand{
    public LoggyPrintCommand(double toLog){
        super(()->DataLogManager.log(Double.toString(toLog)));
    }
    public LoggyPrintCommand(int toLog){
        super(()->DataLogManager.log(Integer.toString(toLog)));
    }
    public LoggyPrintCommand(Object toLog){
        super(()->DataLogManager.log(toLog.toString()));
    }
    public LoggyPrintCommand(String toLog){
        super(()->DataLogManager.log(toLog));
    }
    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
