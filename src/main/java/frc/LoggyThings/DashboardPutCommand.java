package frc.LoggyThings;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class DashboardPutCommand extends InstantCommand{
    public DashboardPutCommand(String key, boolean value){
        super(()-> SmartDashboard.putBoolean(key, value));
    } 
    public DashboardPutCommand(String key, Boolean[] value){
        super(()-> SmartDashboard.putBooleanArray(key, value));
    } 
    public DashboardPutCommand(String key, boolean[] value){
        super(()-> SmartDashboard.putBooleanArray(key, value));
    } 
    public DashboardPutCommand(String key, Sendable value){
        super(()-> SmartDashboard.putData(key, value));
    } 
    public DashboardPutCommand(Sendable value){
        super(()-> SmartDashboard.putData(value));
    } 
    public DashboardPutCommand(String key, double value){
        super(()-> SmartDashboard.putNumber(key, value));
    } 
    public DashboardPutCommand(String key, Double[] value){
        super(()-> SmartDashboard.putNumberArray(key, value));
    } 
    public DashboardPutCommand(String key, double[] value){
        super(()-> SmartDashboard.putNumberArray(key, value));
    } 
    public DashboardPutCommand(String key, byte[] value){
        super(()-> SmartDashboard.putRaw(key, value));
    } 
    public DashboardPutCommand(String key, String value){
        super(()-> SmartDashboard.putString(key, value));
    } 
    public DashboardPutCommand(String key, String[] value){
        super(()-> SmartDashboard.putStringArray(key, value));
    } 
}
