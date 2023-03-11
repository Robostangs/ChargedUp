package frc.LoggyThings;

import java.util.EnumSet;
import java.util.HashMap;

import com.ctre.phoenix.sensors.WPI_CANCoder;

public class LoggyWPI_CanCoder extends WPI_CANCoder implements ILoggyMotor{

    private EnumSet<ILoggyMotor.LogItem> mLogLevel = EnumSet.noneOf(ILoggyMotor.LogItem.class);
    private HashMap<LogItem, DataLogEntryWithHistory> mDataLogEntries = new HashMap<LogItem, DataLogEntryWithHistory>();
    private long mLogPeriod = 100000;// default to 100ms (unit is microseconds)
    private long lastLogTime = (long)Math.abs(Math.random()*100000);
    private String mLogPath;

    public LoggyWPI_CanCoder(int deviceNumber, String canbus, String logPath, EnumSet<ILoggyMotor.LogItem> logLevel) {
        super(deviceNumber, canbus);
        mLogPath = logPath;
        setLogLevel(logLevel);
        LoggyThingManager.getInstance().registerLoggyMotor(this);
    }

    public LoggyWPI_CanCoder(int deviceNumber, String logPath, EnumSet<ILoggyMotor.LogItem> logLevel) {
        this(deviceNumber, logPath, "", logLevel);
    }

    public LoggyWPI_CanCoder(int deviceNumber, String logPath) {
        this(deviceNumber, logPath, "", ILoggyMotor.LogItem.LOGLEVEL_DEFAULT);
    }

    public LoggyWPI_CanCoder(int deviceNumber) {
        this(deviceNumber, "", "", ILoggyMotor.LogItem.LOGLEVEL_DEFAULT);
    }

    @Override
    public void writeToLog() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void setMinimumLogPeriod(double logPeriodSeconds) {
        mLogPeriod = (long) (logPeriodSeconds * 1e6);
    }

    @Override
    public void setLogLevel_internal(EnumSet<LogItem> logLevel) {
        mLogLevel = logLevel;        
    }

    @Override
    public EnumSet<LogItem> getLogLevel() {
        return mLogLevel;
    }

    @Override
    public String getLogPath() {
        return mLogPath;
    }

    @Override
    public HashMap<LogItem, DataLogEntryWithHistory> getDataLogEntries() {
        return mDataLogEntries;
    }
    
}
