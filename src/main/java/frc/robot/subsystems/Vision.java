package frc.robot.subsystems;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils;

public class Vision extends SubsystemBase{
    
    private static Vision Instance;

    private final NetworkTable mLimeLight = NetworkTableInstance.getDefault().getTable("limelight");
    private DoubleArraySubscriber  position = mLimeLight.getDoubleArrayTopic("botpose").subscribe(new double[] {});;
     
    private final NetworkTable mLeftLimelight = NetworkTableInstance.getDefault().getTable("leftLimelight");
    private final NetworkTable mRightLimelight = NetworkTableInstance.getDefault().getTable("rightLimelight");

    public static Vision getInstance() {
        if (Instance == null) {
            Instance = new Vision();
        }
        return Instance;
    }

    public Utils.Vector3D getPosition(String Limelight) {
        if(Limelight == "leftLimelight") {
            return fromAT(mLeftLimelight);
        } else if(Limelight == "rightLimelight") {
            return fromAT(mRightLimelight);
        } else {
            return new Utils.Vector3D(0, 0, 0);
        }
    }

    /**
     * Calculates 3 Dimensional Coordinates from the information of the limelight
     * @param tx
     *            the horizontal angle of the target from the center of the limelight camera
     * @param ty
     *            the vertical angle of the target from the center of the limelight camera
     * @param distanceX
     *            The horizontal distance from the camera to the target
     * @param distanceY
     *             the vertical distance from the camera to the target
     */
    // public Utils.Vector3D fromLL(NetworkTable nt) {
    //     double tx = nt.getEntry("tx").getDouble(0);
    //     double ty = nt.getEntry("ty").getDouble(0);
    //     double distanceX = Constants.Vision.kTargetHeightDelta / (Math.tan(Utils.degToRad(ty + Constants.Vision.kLimelightVerticalAngle)));
    //     double distanceY = distanceY;

    //     return new Utils.Vector3D(distanceX * Math.acos(tx), distanceY, distanceX * Math.asin(tx));
    // }

    public Utils.Vector3D fromAT(NetworkTable nt) {
        double[] positions = position.get();
        return new Utils.Vector3D(positions[0], positions[1], positions[2]);
    }
}
