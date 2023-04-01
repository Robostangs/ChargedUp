package frc.LoggyThings;

import java.util.EnumSet;
import java.util.HashMap;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.util.WPIUtilJNI;

public class LoggyWPI_TalonFX extends WPI_TalonFX implements ILoggyMotor {
    private EnumSet<ILoggyMotor.LogItem> mLogLevel = EnumSet.noneOf(ILoggyMotor.LogItem.class);
    private HashMap<LogItem, DataLogEntryWithHistory> mDataLogEntries = new HashMap<LogItem, DataLogEntryWithHistory>();
    private long mLogPeriod = 100000;// default to 100ms (unit is microseconds)
    private long lastLogTime = (long)Math.abs(Math.random()*100000);
    private String mLogPath;

    public LoggyWPI_TalonFX(int deviceNumber, String logPath, String canbus, EnumSet<ILoggyMotor.LogItem> logLevel) {
        super(deviceNumber, canbus);// actually make motor controller
        mLogPath = logPath;
        setLogLevel(logLevel);
        LoggyThingManager.getInstance().registerLoggyMotor(this);
    }

    public LoggyWPI_TalonFX(int deviceNumber, String logPath, EnumSet<ILoggyMotor.LogItem> logLevel) {
        this(deviceNumber, logPath, "", logLevel);
    }

    public LoggyWPI_TalonFX(int deviceNumber, String logPath) {
        this(deviceNumber, logPath, "", ILoggyMotor.LogItem.LOGLEVEL_DEFAULT);
    }

    public LoggyWPI_TalonFX(int deviceNumber) {
        this(deviceNumber, "/loggyMotors/" + String.valueOf(deviceNumber) + "/", "",
                ILoggyMotor.LogItem.LOGLEVEL_DEFAULT);
    }

    @Override
    public String getLogPath() {
        return mLogPath;
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
    public HashMap<LogItem, DataLogEntryWithHistory> getDataLogEntries() {
        return mDataLogEntries;
    }

    @Override
    public void writeToLog() {
        // Slow down if commanded by manager
        long logPeriod = Long.max(LoggyThingManager.getInstance().getMinGlobalLogPeriod(), mLogPeriod);
        long now = WPIUtilJNI.now();
        if ((now - logPeriod) > lastLogTime) {

            // Only things allowed by the local log level are keys in the datalog entries
            EnumSet<LogItem> potentialLogItems = EnumSet.copyOf(mDataLogEntries.keySet());

            // Only things allowed by the global log level
            potentialLogItems.retainAll(LoggyThingManager.getInstance().getGlobalMaxLogLevel());
            potentialLogItems.removeAll(LogItem.SET_FUNCTION_CALLS);// a set function call, not a periodic status value
            if((getControlMode() == ControlMode.PercentOutput)||(getControlMode() == ControlMode.Follower)||(getControlMode() == ControlMode.MusicTone)||(getControlMode() == ControlMode.Current)||(getControlMode() == ControlMode.Current)){
                potentialLogItems.removeAll(LogItem.PID_LOG_ADDITIONS);
            }
            for (LogItem thisLogItem : potentialLogItems) {
                DataLogEntryWithHistory thisEntry = mDataLogEntries.get(thisLogItem);
                switch (thisLogItem) {
                    case OUTPUT_PERCENT:
                        thisEntry.logDoubleIfChanged(getMotorOutputPercent(), now);
                        break;
                    case FAULTS:
                        Faults faults = new Faults();
                        getFaults(faults);
                        thisEntry.logStringIfChanged(faults.toString(), now);
                        break;
                    case FORWARD_LIMIT_SWITCH:
                        thisEntry.logBooleanIfChanged(getSensorCollection().isFwdLimitSwitchClosed() == 1, now);
                        break;
                    case REVERSE_LIMIT_SWITCH:
                        thisEntry.logBooleanIfChanged(getSensorCollection().isRevLimitSwitchClosed() == 1, now);
                        break;
                    case SELECTED_SENSOR_POSITION:
                        thisEntry.logDoubleIfChanged(getSelectedSensorPosition(), now);
                        break;
                    case SELECTED_SENSOR_VELOCITY:
                        thisEntry.logDoubleIfChanged(getSelectedSensorVelocity(), now);
                        break;
                    case STATOR_CURRENT:
                        thisEntry.logDoubleIfChanged(getStatorCurrent(), now);
                        break;
                    case SUPPLY_CURRENT:
                        thisEntry.logDoubleIfChanged(getSupplyCurrent(), now);
                        break;
                    case BUS_VOLTAGE:
                        thisEntry.logDoubleIfChanged(getBusVoltage(), now);
                        break;
                    case TEMPERATURE:
                        thisEntry.logDoubleIfChanged(getTemperature(), now);
                        break;
                    case INTEGRATED_SENSOR_ABSOLUTE_POSITION:
                        thisEntry.logDoubleIfChanged(getSensorCollection().getIntegratedSensorAbsolutePosition(), now);
                        break;
                    case HAS_RESET:
                        thisEntry.logBooleanIfChanged(hasResetOccurred(), now);
                        break;
                    case CLOSED_LOOP_ERROR:
                        thisEntry.logDoubleIfChanged(getClosedLoopError(), now);
                        break;
                    case INTEGRAL_ACCUMULATOR:
                        thisEntry.logDoubleIfChanged(getIntegralAccumulator(), now);
                        break;
                    case ERROR_DERIVATIVE:
                        thisEntry.logDoubleIfChanged(getErrorDerivative(), now);
                        break;
                    case CLOSED_LOOP_TARGET:
                        thisEntry.logDoubleIfChanged(getClosedLoopTarget(), now);
                        break;
                    case OUTPUT_VOLTAGE:
                        thisEntry.logDoubleIfChanged(getMotorOutputVoltage(), now);
                        break;
                    case INTEGRATED_SENSOR_POSITION:
                        thisEntry.logDoubleIfChanged(getSensorCollection().getIntegratedSensorPosition(), now);
                        break;
                    case INTEGRATED_SENSOR_VELOCITY:
                        thisEntry.logDoubleIfChanged(getSensorCollection().getIntegratedSensorVelocity(), now);
                        break;
                    default:
                        break;
                }
            }
            lastLogTime = WPIUtilJNI.now();
        }
    }

    // set function calls
    @Override
    public void set(double speed) {
        set(ControlMode.PercentOutput,speed);
    }

    @Override
    public void set(ControlMode mode, double value) {
        set(mode, value, DemandType.Neutral, 0);
    }

    boolean justFailed = false;

    @Override
    public void set(ControlMode mode, double demand0, DemandType demand1Type, double demand1) {
        super.set(mode, demand0, demand1Type, demand1);
        try {// Don't jeopardize robot functionality
             // Filter the 4 potential log items down to the ones allowed here
            EnumSet<LogItem> potentialLogItems = EnumSet.of(LogItem.SET_FUNCTION_CONTROL_MODE,
                    LogItem.SET_FUNCTION_VALUE, LogItem.SET_FUNCTION_DEMAND_TYPE, LogItem.SET_FUNCTION_DEMAND);
            potentialLogItems.retainAll(mDataLogEntries.keySet());
            potentialLogItems.retainAll(LoggyThingManager.getInstance().getGlobalMaxLogLevel());
            long now = WPIUtilJNI.now();
            for (LogItem thisLogItem : potentialLogItems) {
                DataLogEntryWithHistory thisEntry = mDataLogEntries.get(thisLogItem);

                switch (thisLogItem) {
                    case SET_FUNCTION_CONTROL_MODE:
                        thisEntry.logStringIfChanged(mode.toString(), now);
                        break;
                    case SET_FUNCTION_VALUE:
                        thisEntry.logDoubleIfChanged(demand0, now);
                        break;
                    case SET_FUNCTION_DEMAND_TYPE:
                        thisEntry.logStringIfChanged(demand1Type.toString(), now);
                        break;
                    case SET_FUNCTION_DEMAND:
                        thisEntry.logDoubleIfChanged(demand1, now);
                    default:
                        break;
                }
            }
            justFailed = false;
        } catch (Exception e) {
            if (!justFailed) {// don't spam log
                e.printStackTrace();
                justFailed = true;
            }
        }

    }

    @Override
    public ErrorCode startMotionProfile(BufferedTrajectoryPointStream stream, int minBufferedPts, ControlMode motionProfControlMode){
        ErrorCode rc = super.startMotionProfile(stream, minBufferedPts, motionProfControlMode);
        try {// Don't jeopardize robot functionality

            if(mDataLogEntries.keySet().contains(LogItem.SET_FUNCTION_CONTROL_MODE) && LoggyThingManager.getInstance().getGlobalMaxLogLevel().contains(LogItem.SET_FUNCTION_CONTROL_MODE)){
                long now = WPIUtilJNI.now();
                mDataLogEntries.get(LogItem.SET_FUNCTION_CONTROL_MODE).logStringIfChanged(motionProfControlMode.toString(), now);
                justFailed = false;
            }
        } catch (Exception e) {
            if (!justFailed) {// don't spam log
                e.printStackTrace();
                justFailed = true;
            }
        }
        return rc;
    }

    @Override
    public void setVoltage(double outputVolts) {
        super.setVoltage(outputVolts);
        try {
            if (mDataLogEntries.keySet().contains(LogItem.SET_VOLTAGE)
                    && LoggyThingManager.getInstance().getGlobalMaxLogLevel().contains(LogItem.SET_VOLTAGE)) {
                mDataLogEntries.get(LogItem.SET_VOLTAGE).logDoubleIfChanged(outputVolts, WPIUtilJNI.now());
                justFailed = false;
            }
        } catch (Exception e) {
            if (!justFailed) {// don;t spam log
                e.printStackTrace();
                justFailed = true;
            }
        }
    }

    @Override
    public ErrorCode setSelectedSensorPosition(double sensorPos) {
        return setSelectedSensorPosition(sensorPos, 0, 0);
    }

    @Override
    public ErrorCode setSelectedSensorPosition(double sensorPos, int pidIdx, int timeoutMs) {
        try {
            if (mDataLogEntries.keySet().contains(LogItem.SET_SELECTED_SENSOR_POSITION)
                    && LoggyThingManager.getInstance().getGlobalMaxLogLevel()
                            .contains(LogItem.SET_SELECTED_SENSOR_POSITION)) {
                mDataLogEntries.get(LogItem.SET_SELECTED_SENSOR_POSITION).logDoubleIfChanged(sensorPos,
                        WPIUtilJNI.now());
                justFailed = false;
            }
        } catch (Exception e) {
            if (!justFailed) {// don't spam log
                e.printStackTrace();
                justFailed = true;
            }
        }
        return super.setSelectedSensorPosition(sensorPos, pidIdx, timeoutMs);
    }

    @Override
    public void setNeutralMode(NeutralMode neutralMode) {
        try {
            if (mDataLogEntries.keySet().contains(LogItem.SET_NEUTRAL_MODE_IS_BRAKE)
                    && LoggyThingManager.getInstance().getGlobalMaxLogLevel()
                            .contains(LogItem.SET_NEUTRAL_MODE_IS_BRAKE)) {
                mDataLogEntries.get(LogItem.SET_NEUTRAL_MODE_IS_BRAKE)
                        .logBooleanIfChanged(neutralMode == NeutralMode.Brake, WPIUtilJNI.now());
                justFailed = false;
            }
        } catch (Exception e) {
            if (!justFailed) {// don't spam log
                e.printStackTrace();
                justFailed = true;
            }
        }
        super.setNeutralMode(neutralMode);
    }

}
