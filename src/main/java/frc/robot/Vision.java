package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Arm;
import frc.robot.Utils.Vector2D;
import frc.robot.Utils.Vector3D;

public class Vision {

    private static Vision Instance;
    private double value;

    private final NetworkTable mLeftLimelight = NetworkTableInstance.getDefault().getTable("limelight-left");
    private DoubleArraySubscriber mLeftPosition = mLeftLimelight.getDoubleArrayTopic("botpose_wpiblue")
            .subscribe(new double[] {});
    private DoubleArraySubscriber mLeftTarget = mLeftLimelight.getDoubleArrayTopic("").subscribe(new double[] {});

    private final NetworkTable mRightLimelight = NetworkTableInstance.getDefault().getTable("limelight-right");
    private DoubleArraySubscriber mRightPosition = mRightLimelight.getDoubleArrayTopic("botpose_wpiblue")
            .subscribe(new double[] {});
    private DoubleArraySubscriber mRightTarget = mLeftLimelight.getDoubleArrayTopic("").subscribe(new double[] {});

    private final NetworkTable mDriverLimelight = NetworkTableInstance.getDefault().getTable("limelight-driver");
    private DoubleSubscriber mObjectTY = mDriverLimelight.getDoubleTopic("ty").subscribe(value);
    private DoubleSubscriber mObjectTX = mDriverLimelight.getDoubleTopic("tx").subscribe(value);

    private double lastLeftMeasurementTimestamp;
    private double lastRightMeasurementTimestamp;

    public enum LimelightState {
        leftLimelight,
        rightLimelight,
        center,
        unknown
    }

    public static Vision getInstance() {
        if (Instance == null) {
            Instance = new Vision();
        }
        return Instance;
    }

    public boolean targetVisible(LimelightState limelight) {
        if (limelight.compareTo(LimelightState.leftLimelight) == 0) {
            return (mLeftLimelight.getEntry("ta").getDouble(0) != 0);
        } else if (limelight.compareTo(LimelightState.rightLimelight) == 0) {
            return (mRightLimelight.getEntry("ta").getDouble(0) != 0);
        } else if (limelight.compareTo(LimelightState.center) == 0) {
            return (mDriverLimelight.getEntry("ta").getDouble(0) > 1.4);
        } else {
            return false;
        }
    }

    public Utils.Vector3D getPosition(LimelightState Limelight) {
        if (Limelight.compareTo(LimelightState.leftLimelight) == 0) {
            return fromAT(mLeftPosition);
        } else if (Limelight.compareTo(LimelightState.rightLimelight) == 0) {
            return fromAT(mRightPosition);
        } else {
            return new Utils.Vector3D(0, 0, 0);
        }
    }

    public void takeSnapshotDriver() {
        mDriverLimelight.getEntry("snapshot").setNumber(1);
    }

    public Rotation2d getRotation(LimelightState Limelight) {
        if (Limelight.compareTo(LimelightState.leftLimelight) == 0) {
            return rotFromAT(mLeftPosition);
        } else if (Limelight.compareTo(LimelightState.rightLimelight) == 0) {
            return rotFromAT(mRightPosition);
        } else {
            return new Rotation2d();
        }
    }

    public void switchPipelinesDrivetrain() {
        if (mDriverLimelight.getEntry("getpipe").getInteger(100) == 1) {
            mDriverLimelight.getEntry("pipeline").setValue("0");
        } else {
            mDriverLimelight.getEntry("pipeline").setValue("1");
        }
    }

    public double getTargetHandX() {
        /*
         * double theta1 = Arm.getInstance().getShoulderAngle();
         * double ty = mObjectTY.get();
         * double value =
         * (Constants.Arm.upperarmLength-Constants.Arm.LimelightCenterToShoulderPivot)
         * Math.sin(Math.toRadians(90 + ty))
         * / Math.sin(Math.toRadians(90 - ty - theta1))
         * +0.07/Math.cos(Math.toRadians(90-theta1));//offset for limelight distance
         * from arm centrline
         * // System.out.println(ty + "," + value + "," + theta1);
         * return value + 0.05;
         */
        return 0;
    }

    public double getDrivetrainAngle() {
        return mObjectTX.get();
    }

    public Utils.Vector3D getLimelightPosition(LimelightState Limelight) {
        if (Limelight.compareTo(LimelightState.leftLimelight) == 1) {
            return forTarget(mLeftTarget);
        } else if (Limelight.compareTo(LimelightState.rightLimelight) == 1) {
            return forTarget(mRightTarget);
        } else {
            return new Utils.Vector3D(100, 100, 100);
        }
    }

    /**
     * Calculates 3 Dimensional Coordinates from the information of the limelight
     * 
     * @param tx
     *                  the horizontal angle of the target from the center of the
     *                  limelight camera
     * @param ty
     *                  the vertical angle of the target from the center of the
     *                  limelight camera
     * @param distanceX
     *                  The horizontal distance from the camera to the target
     * @param distanceY
     *                  the vertical distance from the camera to the target
     */
    // public Utils.Vector3D fromLL(NetworkTable nt) {
    // double tx = nt.getEntry("tx").getDouble(0);
    // double ty = nt.getEntry("ty").getDouble(0);
    // double distanceX = Constants.Vision.kTargetHeightDelta /
    // (Math.tan(Utils.degToRad(ty + Constants.Vision.kLimelightVerticalAngle)));
    // double distanceY = distanceY;

    // return new Utils.Vector3D(distanceX * Math.acos(tx), distanceY, distanceX *
    // Math.asin(tx));
    // }

    public double getCenterLimelightTX() {
        return mObjectTX.get();
    }

    public Utils.Vector3D fromAT(DoubleArraySubscriber sub) {
        double[] positions = sub.get();
        return new Utils.Vector3D(positions[0], positions[1], positions[2]);
    }

    public Rotation2d rotFromAT(DoubleArraySubscriber sub) {
        double[] positions = sub.get();
        return Rotation2d.fromDegrees(positions[5]);
    }

    public Utils.Vector3D forTarget(DoubleArraySubscriber sub) {
        double[] positions = sub.get();
        return new Utils.Vector3D(positions[0], positions[1], positions[2]);
    }

    public Optional<LimelightMeasurement> getNewLeftMeasurement() {
        double[] positions = mLeftPosition.get();
        if (!targetVisible(LimelightState.leftLimelight)) {
            return Optional.empty();
        }
        double timestamp = delayToTime(positions[6]);
        if ((timestamp - lastLeftMeasurementTimestamp) < Constants.Swerve.Odometry.MIN_TIME_BETWEEN_LL_UPDATES_MS) {
            return Optional.empty();
        }
        lastLeftMeasurementTimestamp = timestamp;

        Vector3D position = fromAT(mLeftPosition);
        Rotation2d rotation = rotFromAT(mLeftPosition);

        return Optional.of(
                new LimelightMeasurement(new Pose2d(position.x, position.y, rotation),
                        timestamp));
    }

    public Optional<LimelightMeasurement> getNewRightMeasurement() {
        double[] positions = mRightPosition.get();
        if (!targetVisible(LimelightState.rightLimelight)) {
            return Optional.empty();
        }
        double timestamp = delayToTime(positions[6]);
        if ((timestamp - lastRightMeasurementTimestamp) < Constants.Swerve.Odometry.MIN_TIME_BETWEEN_LL_UPDATES_MS) {
            return Optional.empty();
        }
        lastRightMeasurementTimestamp = timestamp;

        Vector3D position = fromAT(mRightPosition);
        Rotation2d rotation = rotFromAT(mRightPosition);

        return Optional.of(
                new LimelightMeasurement(new Pose2d(position.x, position.y, rotation),
                        timestamp));
    }

    public Vector2D objectPosition() {
        return new Vector2D(mObjectTX.get(), mObjectTY.get());
    }
    
    private static double delayToTime(double delay) {
        return (Timer.getFPGATimestamp() - (delay / 1000));
    }

    public class LimelightMeasurement {
        public final Pose2d mPose;
        public final double mTime;

        public LimelightMeasurement(Pose2d pose, double time) {
            mPose = pose;
            mTime = time;
        }
    }

    public void switchPipelines(LimelightState state) {
        if (state.compareTo(LimelightState.leftLimelight) == 0) {
            if (mLeftLimelight.getValue("pipeline").getInteger() == 0) {
                mLeftLimelight.putValue("pipeline", NetworkTableValue.makeInteger(1));
            } else {
                mLeftLimelight.putValue("pipeline", NetworkTableValue.makeInteger(0));
            }
        } else if (state.compareTo(LimelightState.rightLimelight) == 0) {
            if (mRightLimelight.getValue("pipeline").getInteger() == 0) {
                mRightLimelight.putValue("pipeline", NetworkTableValue.makeInteger(1));
            } else {
                mRightLimelight.putValue("pipeline", NetworkTableValue.makeInteger(0));
            }
        } else if (state.compareTo(LimelightState.center) == 0) {
            if (mDriverLimelight.getValue("pipeline").getInteger() == 0) {
                mDriverLimelight.putValue("pipeline", NetworkTableValue.makeInteger(1));
            } else {
                mDriverLimelight.putValue("pipeline", NetworkTableValue.makeInteger(0));
            }
        }
    }
    public Vector2D calculateAndPrintGamePiecePosition(){
        Vector2D mVector2d = Vision.getInstance().objectPosition();
        
        double mTargetXFromOrigin = Math.abs(((degSin(90+mVector2d.y) * Constants.Arm.upperarmLength)
            / degSin(90-mVector2d.y-Arm.getInstance().getShoulderPositionFromMotor())));
        double mTargetY = (mTargetXFromOrigin * degTan(-mVector2d.x));
        double mTargetX = mTargetXFromOrigin- (Arm.getInstance().getHandPositionX())-0.06;

        // Vector2D mVector2d = Vision.getInstance().objectPosition();

        SmartDashboard.putNumber("Auto Grab/Target/X", mTargetX);
        SmartDashboard.putNumber("Auto Grab/Target/Y", mTargetY);
        return new Vector2D(mTargetX, mTargetY);
    }

    public double degSin(double value) {
        return Math.sin(Math.toRadians(value));
    }

    public double degCos(double value) {
        return Math.cos(Math.toRadians(value));
    }

    public double degTan(double value) {
        return Math.tan(Math.toRadians(value));
    }
}
