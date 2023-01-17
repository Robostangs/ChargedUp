package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.LoggyThings.LoggyWPI_TalonFX;
import frc.robot.Constants;
import frc.robot.Utils;
import frc.robot.Vision;
import frc.robot.Utils.Vector2D;
import frc.robot.Vision.LimelightState;

public class Drivetrain extends SubsystemBase{
    
    private static Drivetrain Instance;
    private Vision mVision; 
    private Vector2D v1, v2;
    private Vector2D current;

    private final static SwerveModulePosition[] blankPositions = {new SwerveModulePosition(0,new Rotation2d(0)), new SwerveModulePosition(0,new Rotation2d(0)), new SwerveModulePosition(0,new Rotation2d(0)), new SwerveModulePosition(0,new Rotation2d(0))};

    private final SwerveModule mLeftFrontModule;
    private final SwerveModule mRightFrontModule;
    private final SwerveModule mLeftBackModule;
    private final SwerveModule mRightBackModule;

    private final LoggyWPI_TalonFX mLeftFrontDrive;
    private final LoggyWPI_TalonFX mRightFrontDrive;
    private final LoggyWPI_TalonFX mLeftBackDrive;
    private final LoggyWPI_TalonFX mRightBackDrive; 

    private final LoggyWPI_TalonFX mleftFrontSteer;
    private final LoggyWPI_TalonFX mRightFrontSteer;
    private final LoggyWPI_TalonFX mLeftBackSteer;
    private final LoggyWPI_TalonFX mRightBackSteer;

    private final AHRS mGyro = new AHRS(SPI.Port.kMXP);

    private ChassisSpeeds mChassisSpeeds = new ChassisSpeeds(0, 0, 0);

    private final SwerveDriveOdometry mOdometry = new SwerveDriveOdometry(Constants.Drivetrain.mKinematics, new Rotation2d(0), blankPositions);
   
    public static Drivetrain getInstance() {
        if (Instance == null) {
            Instance = new Drivetrain();
        }
        return Instance;
    }

    private Drivetrain() {
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

        mLeftFrontDrive = new LoggyWPI_TalonFX(Constants.Drivetrain.frontLeftDriveID, "/Drivetrain/Left/Front/Drive/");
        mRightFrontDrive = new LoggyWPI_TalonFX(Constants.Drivetrain.frontRightDriveID, "/Drivetrain/Right/Front/Drive/");
        mLeftBackDrive = new LoggyWPI_TalonFX(Constants.Drivetrain.backLeftDriveID, "/Drivetrain/Left/Back/Drive/");
        mRightBackDrive = new LoggyWPI_TalonFX(Constants.Drivetrain.backRightDriveID, "/Drivetrain/Right/Back/Drive/");

        mleftFrontSteer = new LoggyWPI_TalonFX(Constants.Drivetrain.frontLeftSteerID, "/Drivetrain/Left/Front/Steer/");
        mRightFrontSteer = new LoggyWPI_TalonFX(Constants.Drivetrain.frontRightSteerID, "/Drivetrain/Right/Front/Steer/");
        mLeftBackSteer = new LoggyWPI_TalonFX(Constants.Drivetrain.backLeftSteerID, "/Drivetrain/Left/Back/Steer/");
        mRightBackSteer = new LoggyWPI_TalonFX(Constants.Drivetrain.backRightSteerID, "/Drivetrain/Right/Back/Steer/");

        mLeftFrontDrive.setNeutralMode(NeutralMode.Coast);
        mRightFrontDrive.setNeutralMode(NeutralMode.Coast);
        mLeftBackDrive.setNeutralMode(NeutralMode.Coast);
        mRightBackDrive.setNeutralMode(NeutralMode.Coast);

        mleftFrontSteer.setNeutralMode(NeutralMode.Brake);
        mRightFrontSteer.setNeutralMode(NeutralMode.Brake);
        mLeftBackSteer.setNeutralMode(NeutralMode.Brake);
        mRightBackSteer.setNeutralMode(NeutralMode.Brake);

        mVision = Vision.getInstance();

        mLeftFrontModule = Mk4iSwerveModuleHelper.createFalcon500(
                tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                   .withSize(2,4)
                   .withPosition(0,0), 
                Mk4iSwerveModuleHelper.GearRatio.L1,
                Constants.Drivetrain.frontLeftDriveID, 
                Constants.Drivetrain.frontLeftSteerID,
                Constants.Drivetrain.frontLeftEncoderID, 
                Constants.Drivetrain.frontLeftEncoderOffset
        );

        mRightFrontModule = Mk4iSwerveModuleHelper.createFalcon500(
                tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                   .withSize(2,4)
                   .withPosition(2,0), 
                Mk4iSwerveModuleHelper.GearRatio.L1,
                Constants.Drivetrain.frontRightDriveID, 
                Constants.Drivetrain.frontRightSteerID,
                Constants.Drivetrain.frontRightEncoderID, 
                Constants.Drivetrain.frontRightEncoderOffset
        );

        mLeftBackModule = Mk4iSwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                   .withSize(2,4)
                   .withPosition(0,4), 
                Mk4iSwerveModuleHelper.GearRatio.L1,
                Constants.Drivetrain.backLeftDriveID, 
                Constants.Drivetrain.backLeftSteerID,
                Constants.Drivetrain.backLeftEncoderID, 
                Constants.Drivetrain.backLeftEncoderOffset
        );

        mRightBackModule = Mk4iSwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                   .withSize(2,4)
                   .withPosition(2,4), 
                Mk4iSwerveModuleHelper.GearRatio.L1,
                Constants.Drivetrain.backRightDriveID, 
                Constants.Drivetrain.backRightSteerID,
                Constants.Drivetrain.backRightEncoderID, 
                Constants.Drivetrain.backRightEncoderOffset
        );
    }

    public void zeroGyro() {
        mGyro.reset();
    }

    public double getGyroAngle() {
        return mGyro.getAngle();
    }

    public Rotation2d getGyroscopeRotation() {
        return Rotation2d.fromDegrees(180 - mGyro.getYaw());
    }

    public void resetOdometry(Pose2d pose) {
        mOdometry.resetPosition(getGyroscopeRotation(), blankPositions, pose);
    }

    public void spin(double power) {
        drive(ChassisSpeeds.fromFieldRelativeSpeeds(0,0,power,getGyroscopeRotation()));
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        mChassisSpeeds = chassisSpeeds;
    }

    public Pose2d getPose() {
        return mOdometry.getPoseMeters();
    }

    public void setModuleStates(SwerveModuleState[] states) {
        mLeftFrontModule.set(states[0].speedMetersPerSecond / Constants.Drivetrain.maxLinearVelocity * Constants.Drivetrain.maxVoltage, states[0].angle.getRadians());
        mRightFrontModule.set(states[0].speedMetersPerSecond / Constants.Drivetrain.maxLinearVelocity * Constants.Drivetrain.maxVoltage, states[0].angle.getRadians());
        mLeftBackModule.set(states[0].speedMetersPerSecond / Constants.Drivetrain.maxLinearVelocity * Constants.Drivetrain.maxVoltage, states[0].angle.getRadians());
        mRightBackModule.set(states[0].speedMetersPerSecond / Constants.Drivetrain.maxLinearVelocity * Constants.Drivetrain.maxVoltage, states[0].angle.getRadians());
        
    }

    public void update() {
        if(mVision.targetVisible(LimelightState.leftLimelight) && mVision.targetVisible(LimelightState.rightLimelight)) {
            v1.set(mVision.getPosition(LimelightState.leftLimelight).x, mVision.getPosition(LimelightState.leftLimelight).y);
            v2.set(mVision.getPosition(LimelightState.rightLimelight).x, mVision.getPosition(LimelightState.rightLimelight).y);
            current.set(getPose().getX(), getPose().getY()); 

            if(Utils.withinRange(v1, current) && Utils.withinRange(v2, current)) {

            } else if(Utils.withinRange(v1, current)) { 

            } else if(Utils.withinRange(v2, current)) {

            } else {
                updateOdometry();
            }

        } else if(mVision.targetVisible(LimelightState.leftLimelight)) {
            v1.set(mVision.getPosition(LimelightState.leftLimelight).x, mVision.getPosition(LimelightState.leftLimelight).y);
            current.set(getPose().getX(), getPose().getY());
            
            if(Utils.withinRange(v1, current)) {

            } else {
                updateOdometry();
            }
        } else if(mVision.targetVisible(LimelightState.rightLimelight)) {
            v2.set(mVision.getPosition(LimelightState.rightLimelight).x, mVision.getPosition(LimelightState.rightLimelight).y);
            current.set(getPose().getX(), getPose().getY());

            if(Utils.withinRange(v2, current)) {
                
            } else {
                updateOdometry();
            }
        } else {
            updateOdometry();
        }
    }

    public void updateOdometry() {
        SwerveModuleState[] states = Constants.Drivetrain.mKinematics.toSwerveModuleStates(mChassisSpeeds);
        mOdometry.update(getGyroscopeRotation(), 
                new SwerveModulePosition[]{ 
                        new SwerveModulePosition(states[0].speedMetersPerSecond, states[0].angle),
                        new SwerveModulePosition(states[1].speedMetersPerSecond, states[1].angle),
                        new SwerveModulePosition(states[2].speedMetersPerSecond, states[2].angle),
                        new SwerveModulePosition(states[3].speedMetersPerSecond, states[3].angle),
                }
        );
    }

    public void updateWithLimelight(Vector2D target) {
        SwerveModuleState[] states = Constants.Drivetrain.mKinematics.toSwerveModuleStates(mChassisSpeeds);
        mOdometry.update(getGyroscopeRotation(), 
                new SwerveModulePosition[]{ 
                        new SwerveModulePosition(states[0].speedMetersPerSecond, states[0].angle),
                        new SwerveModulePosition(states[1].speedMetersPerSecond, states[1].angle),
                        new SwerveModulePosition(states[2].speedMetersPerSecond, states[2].angle),
                        new SwerveModulePosition(states[3].speedMetersPerSecond, states[3].angle),
                }
        );
    }
    
    @Override
    public void periodic() {
        update();
        SwerveModuleState[] states = Constants.Drivetrain.mKinematics.toSwerveModuleStates(mChassisSpeeds);
        setModuleStates(states);
    }
}
