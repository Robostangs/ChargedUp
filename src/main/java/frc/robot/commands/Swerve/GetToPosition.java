package frc.robot.commands.Swerve;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.TreeMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Vision;
import frc.robot.Utils.Vector3D;
import frc.robot.Vision.LimelightState;
import frc.robot.autos.Translate;
import frc.robot.subsystems.Swerve;
import frc.robot.Utils;

public class GetToPosition extends CommandBase {
    Swerve mDrivetrain = Swerve.getInstance();
    private boolean mHoldingCone;
    private static final ArrayList<Double> kConePositions = new ArrayList<Double>();
    private static final ArrayList<Double> kCubePositions = new ArrayList<Double>();
    double mSpeed;
    Utils.Vector2D setPosition;

    public GetToPosition(double speed, boolean isCone) {
        setName("Straighten Against Wall");
        mSpeed = speed;
        mHoldingCone = isCone;

        kConePositions.add(0.519);
        kConePositions.add(1.636);
        kConePositions.add(2.195);
        kConePositions.add(3.130);
        kConePositions.add(3.872);
        kConePositions.add(4.982);

        kCubePositions.add(0.849 + (0.459/2));
        kCubePositions.add(2.525 + (0.459/2));
        kCubePositions.add(4.202 + (0.459/2));
    }

    @Override
    public void initialize() {
        Pose2d currentPos = mDrivetrain.getPose();
        if(mHoldingCone) {
            setPosition=new Utils.Vector2D(kConePositions.get(0), 1.61);
            for(int i = 1; i < kConePositions.size(); i++) {
                if(kConePositions.get(i-1) <= currentPos.getX() && kConePositions.get(i) >= currentPos.getX()) {
                    if((kConePositions.get(i-1) - currentPos.getX()) <= (kConePositions.get(i) - currentPos.getX())) {
                        if(currentPos.getY() > Constants.fieldLength/2) {
                            setPosition = new Utils.Vector2D(kConePositions.get(i-1), Constants.fieldLength - 1.61);
                        } else {
                            setPosition = new Utils.Vector2D(kConePositions.get(i-1), 1.61);
                        }
                    } else {
                        if(currentPos.getY() > Constants.fieldLength/2) {
                            setPosition = new Utils.Vector2D(kConePositions.get(i), Constants.fieldLength - 1.61);
                        } else {
                            setPosition = new Utils.Vector2D(kConePositions.get(i), 1.61);
                        }
                    }
                }
            }
        } else {
            for(int i = 1; i < kCubePositions.size(); i++) {
                setPosition=new Utils.Vector2D(kCubePositions.get(0), 1.61);
                if(kCubePositions.get(i-1) <= currentPos.getX() && kCubePositions.get(i) >= currentPos.getX()) {
                    if((kCubePositions.get(i-1) - currentPos.getX()) <= (kCubePositions.get(i) - currentPos.getX())) {
                        if(currentPos.getY() > Constants.fieldLength/2) {
                            setPosition = new Utils.Vector2D(kCubePositions.get(i-1), Constants.fieldLength - 1.61);
                        } else {
                            setPosition = new Utils.Vector2D(kCubePositions.get(i-1), 1.61);
                        }
                    } else {
                        if(currentPos.getY() > Constants.fieldLength/2) {
                            setPosition = new Utils.Vector2D(kCubePositions.get(i), Constants.fieldLength - 1.61);
                        } else {
                            setPosition = new Utils.Vector2D(kCubePositions.get(i), 1.61);
                        }
                    }
                }
            }
        }
        new Translate(mDrivetrain, setPosition).schedule();
    }

}
