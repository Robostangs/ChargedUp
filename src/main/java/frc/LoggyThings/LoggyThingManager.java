package frc.LoggyThings;

import java.util.ArrayList;
import java.util.EnumSet;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

public class LoggyThingManager {
    private static LoggyThingManager instance = null;
    private ArrayList<ILoggyMotor> mMotorList = new ArrayList<ILoggyMotor>();
    private EnumSet<ILoggyMotor.LogItem> mGlobalMaxLogLevel = ILoggyMotor.LogItem.LOGLEVEL_EVERYTHING;
    private long mMinGlobalLogPeriod = 0;// microseconds between possible logs, either user period or slowed down if
                                         // disabled
    private long mUserMinGlobalLogPeriod = 0; // user set min period, not overwritten if disabled

    LoggyThingManager() {
        System.out.println("Loggy Thing Manager Initialized");
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog(), true);
    }
    
    public long getMinGlobalLogPeriod() {
        return mMinGlobalLogPeriod;
    }

    public void setMinGlobalLogPeriod(double minLogPeriodSeconds) {
        mUserMinGlobalLogPeriod = ((long) (minLogPeriodSeconds * 1e6));
    }

    public void setGlobalMaxLogLevel(EnumSet<ILoggyMotor.LogItem> globalMaxLogLevel) {
        mGlobalMaxLogLevel = globalMaxLogLevel;
    }

    public EnumSet<ILoggyMotor.LogItem> getGlobalMaxLogLevel() {
        return mGlobalMaxLogLevel;
    }

    public static LoggyThingManager getInstance() {
        if (instance == null)
            instance = new LoggyThingManager();
        return instance;
    }

    public void registerLoggyMotor(ILoggyMotor loggyMotor) {
        mMotorList.add(loggyMotor);
    }

    boolean justFailed = false;

    // Call this from robot periodic, don't crash the robot
    public void periodic() {
        try {
            // Slow down logging if disabled
            mMinGlobalLogPeriod = DriverStation.isEnabled() ? mUserMinGlobalLogPeriod : 10000000;
            for (ILoggyMotor iLoggyMotor : mMotorList) {
                iLoggyMotor.writeToLog();
            }
            justFailed = false;
        } catch (Exception e) {
            if(!justFailed){
                e.printStackTrace();
                justFailed = true;
            }
        }

    }
}
