package frc.robot.commands.Swerve;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.TreeMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Vision;
import frc.robot.Utils.Vector3D;
import frc.robot.Vision.LimelightState;
import frc.robot.autos.translate;
import frc.robot.subsystems.Hand;
import frc.robot.subsystems.Swerve;
import frc.robot.Utils;

public class GetToPosition extends CommandBase {
    Swerve mDrivetrain = Swerve.getInstance();
    Hand mHand = Hand.getInstance();
    private boolean mHoldingCone;
    private static final ArrayList<Double> kConePositions = new ArrayList<Double>();
    private static final ArrayList<Double> kCubePositions = new ArrayList<Double>();
    Utils.Vector3D setPosition;

    public GetToPosition() {
        setName("Straighten Against Wall");
        addRequirements(mDrivetrain);

        kConePositions.add(0.512);
        kConePositions.add(1.630);
        kConePositions.add(2.189);
        kConePositions.add(3.306);
        kConePositions.add(3.865);
        kConePositions.add(4.983);

        kCubePositions.add(1.071);
        kCubePositions.add(2.747);
        kCubePositions.add(4.424);
    }

    @Override
    public void initialize() {
        mHoldingCone = mHand.getHolding();
        Pose2d currentPos = mDrivetrain.getPose();
        if(mHoldingCone) {
            setPosition=new Utils.Vector3D(1.79, kConePositions.get(0), 180);
            if(kConePositions.get(kConePositions.size() - 1) < currentPos.getY()) {
                if(currentPos.getX() > Constants.fieldLength/2) {
                    setPosition = new Utils.Vector3D(Constants.fieldLength - 1.79,kConePositions.get(kConePositions.size() -1),  0);
                } else {
                    setPosition = new Utils.Vector3D(1.79,kConePositions.get(kConePositions.size() -1), 180);
                }
            }

            for(int i = 1; i < kConePositions.size(); i++) {
                if(kConePositions.get(i-1) <= currentPos.getY() && kConePositions.get(i) >= currentPos.getY()) {
                    if((kConePositions.get(i-1) - currentPos.getY()) <= (kConePositions.get(i) - currentPos.getY())) {
                        if(currentPos.getX() > Constants.fieldLength/2) {
                            setPosition = new Utils.Vector3D(Constants.fieldLength - 1.79, kConePositions.get(i-1), 0);
                        } else {
                            setPosition = new Utils.Vector3D(1.79,kConePositions.get(i-1), 180);
                        }
                    } else {
                        if(currentPos.getX() > Constants.fieldLength/2) {
                            setPosition = new Utils.Vector3D(Constants.fieldLength - 1.79,kConePositions.get(i),  0);
                        } else {
                            setPosition = new Utils.Vector3D( 1.79, kConePositions.get(i),180);
                        }
                    }
                }
            }
        } else {
            for(int i = 1; i < kCubePositions.size(); i++) {
                setPosition=new Utils.Vector3D(1.79, kCubePositions.get(0), 0);
                if(kCubePositions.get(kCubePositions.size() - 1) < currentPos.getY()) {
                    if(currentPos.getX() > Constants.fieldLength/2) {
                        setPosition = new Utils.Vector3D(Constants.fieldLength - 1.79,kCubePositions.get(kCubePositions.size() -1),  0);
                    } else {
                        setPosition = new Utils.Vector3D(1.79,kCubePositions.get(kCubePositions.size() -1), 180);
                    }
                }

                if(kCubePositions.get(i-1) <= currentPos.getY() && kCubePositions.get(i) >= currentPos.getY()) {
                    if((kCubePositions.get(i-1) - currentPos.getY()) <= (kCubePositions.get(i) - currentPos.getY())) {
                        if(currentPos.getX() > Constants.fieldLength/2) {
                            setPosition = new Utils.Vector3D(Constants.fieldLength - 1.79,kCubePositions.get(i-1),  0);
                        } else {
                            setPosition = new Utils.Vector3D( 1.79,kCubePositions.get(i-1), 180);
                        }
                    } else {
                        if(currentPos.getX() > Constants.fieldLength/2) {
                            setPosition = new Utils.Vector3D( Constants.fieldLength - 1.79,kCubePositions.get(i), 0);
                        } else {
                            setPosition = new Utils.Vector3D( 1.79,kCubePositions.get(i), 180);
                        }
                    }
                }
            }
        }
        SmartDashboard.putString("CurrentPosition", mDrivetrain.getPose().getY() + " " + mDrivetrain.getPose().getY() + " " + mDrivetrain.getPose().getRotation().getDegrees() + " ");
        SmartDashboard.putString("Position", setPosition.x + " " + setPosition.y + " " + setPosition.z + " ");
        // setPosition = new Vector3D(1.81, 4.43, 180 );
        new ParallelDeadlineGroup(new translate(mDrivetrain, setPosition).withTimeout(3), new InstantCommand(() -> System.out.println("I am a smart idiot"))).schedule();
    }

}
