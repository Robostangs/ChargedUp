package frc.robot.subsystems;

import java.util.Optional;
import java.util.Vector;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.Vision;
import frc.robot.Vision.LimelightMeasurement;

public class Swerve extends SubsystemBase {
    private SwerveDrivePoseEstimator swerveOdometry;
    public SwerveModule[] mSwerveMods;
    private Field2d mField = new Field2d();
    private Pigeon2 mGyro;
    private Vision mVision = Vision.getInstance();

    // The Swerve class should not hold the vision systems, this is a great way to
    // end up in dependecy hell
    // Use double suppliers or something instead and keep vision in robot
    // container...
    public static Swerve mInstance;

    public static Swerve getInstance() {

        if (mInstance == null) {
            mInstance = new Swerve();
        }
        return mInstance;
    }

    public Swerve() {
        mGyro = new Pigeon2(3);
        SmartDashboard.putData("field", mField);
        zeroGyro();

        mSwerveMods = new SwerveModule[] {
                new SwerveModule(1, Constants.Swerve.Mod0.constants),
                new SwerveModule(0, Constants.Swerve.Mod1.constants),
                new SwerveModule(2, Constants.Swerve.Mod2.constants),
                new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        /*
         * By pausing init for a second before setting module offsets, we avoid a bug
         * with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();

        swerveOdometry = new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics,
            getYaw(),
            getModulePositions(),
            new Pose2d(),
            Constants.Swerve.Odometry.STATE_STANDARD_DEVS,
            Constants.Swerve.Odometry.VISION_STANDARD_DEVS);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getYaw())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public Pose2d getPose() {
        return swerveOdometry.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(Rotation2d.fromDegrees(getGyroAngle()), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    /**
     * Reset Gryo to 0 degrees
     */
    public void zeroGyro() {
        mGyro.setYaw(0);
    }

    public void setGyro(double angle) {
        mGyro.setYaw(angle);
    }

    public double getGyroAngle() {
        return (Math.abs(mGyro.getYaw()) % 360);
    }

    public double getRawGyroAngle() {
        return mGyro.getYaw();
    }

    

    public double getPitchAngle() {
        return mGyro.getRoll() + 1.31;
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - mGyro.getYaw())
                : Rotation2d.fromDegrees(mGyro.getYaw());
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    public void updateOdometry() {
        updateWithSwerveStates();
        updateWithLimelight();
    }

    public void updateWithSwerveStates() {
        swerveOdometry.update(getYaw(), getModulePositions());
    }

    public void updateOdometryManual(double x, double y, double angle) {
                swerveOdometry.resetPosition(Rotation2d.fromDegrees(getGyroAngle()), getModulePositions(), new Pose2d(x, y, Rotation2d.fromDegrees(angle)));
    }

    /**
     * Turn all wheels diagonal so Swerve is locked
     */
    public void lockPosition() {
        SwerveModuleState[] lockStates = {
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(315)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(225)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(135)),
        };
        setModuleStates(lockStates);
    }

    private void updateWithLimelight() {
        Optional<LimelightMeasurement> leftMeasurement = mVision.getNewLeftMeasurement();
        if (!leftMeasurement.isEmpty()) {
            swerveOdometry.addVisionMeasurement(leftMeasurement.get().mPose, leftMeasurement.get().mTime);
        }

        Optional<LimelightMeasurement> rightMeasurement = mVision.getNewRightMeasurement();
        if (!rightMeasurement.isEmpty()) {
            swerveOdometry.addVisionMeasurement(rightMeasurement.get().mPose, rightMeasurement.get().mTime);
        }
    }

    @Override
    public void periodic() {
        updateOdometry();
        SmartDashboard.putString("CurrentPosition",
                getPose().getX() + " " + getPose().getY() + " " + getPose().getRotation().getDegrees() + " ");
        SmartDashboard.putNumber("angle", getGyroAngle());

        SmartDashboard.putNumber("Front Right Angle", mSwerveMods[0].getCanCoder().getDegrees());
        SmartDashboard.putNumber("Front Left Angle", mSwerveMods[1].getCanCoder().getDegrees());
        SmartDashboard.putNumber("Back Left Angle", mSwerveMods[2].getCanCoder().getDegrees());
        SmartDashboard.putNumber("Back Right Angle", mSwerveMods[3].getCanCoder().getDegrees());

        mField.setRobotPose(getPose());
    }
    
    public void addFieldObj(Trajectory trajectory) {
        Vector<Pose2d> poses = new Vector<Pose2d>();
        poses.add(trajectory.getInitialPose());
        for (int i = 0; i < trajectory.getStates().size(); i++) {
            if (i % 10 == 0 && trajectory.getStates().get(i).poseMeters != poses.lastElement()) {
                poses.add(trajectory.getStates().get(i).poseMeters);
            }
        }
        // for (State state : trajectory.getStates()) {
        //     poses.add(state.poseMeters);
        // }
        mField.getObject(Constants.AutoConstants.kFieldObjectName).setPoses(poses);
    }

    public Field2d getField() {
        return mField;
    }

    public void outputModuleStates(SwerveModuleState[] states) {
        for (int i = 0; i < states.length; i++) {
            mSwerveMods[i].setDesiredState(states[i], false);
        }
    }
}
