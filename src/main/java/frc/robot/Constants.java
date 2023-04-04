package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

//Numbering system for drivetrain: 0 - front right, 1 - front left, 2 - back left, 3 - back right
//0.42545 + 0.254/2

public final class Constants {
    public static final double stickDeadband = 0.1;
    public static final double fieldLength = 16.54;

    public static final class Swerve {
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final double kRange = 20;

        public static class CustomDeadzone {

          public static final double kLowerLimitExpFunc = 0.1;
          public static final double kUpperLimitExpFunc = 0.5;
          public static final double kUpperLimitLinFunc = 1;
    
          public static final double kExpFuncConstant = 0.3218;
          public static final double kExpFuncBase = 12.5;
          public static final double kExpFuncMult = 0.25;
    
          public static final double kLinFuncMult = 0.876;
          public static final double kLinFuncOffset = 0.5;
          public static final double kLinFuncConstant = 0.562;
    
          public static final double kNoSpeed = 0;

          public static final double kJoyStickDeadZone = 0.05;
        }

        public static final COTSFalconSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
            COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L1);

        /* Drivetrain Constants */
        public static final double trackWidth = 0.501; //TODO: This must be tuned to specific robot
        public static final double wheelBase = 0.615; //TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.32 / 12); //TODO: This must be tuned to specific robot
        public static final double driveKV = (1.51 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
        public static double testSpeed = 0.2;
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        public static final double targetOffset = 0;

        /* Module Specific Constants */
        /* Front Right Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 11;
            public static final int angleMotorID = 10;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(9.492);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Left Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 20;
            public static final int angleMotorID = 21;
            public static final int canCoderID = 22;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(250.40);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 30;
            public static final int angleMotorID = 31;
            public static final int canCoderID = 32;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(201.445 );
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 40;
            public static final int angleMotorID = 41;
            public static final int canCoderID = 42;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(95.086);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        public static final class balancePID {
            public static final double kP = 0.012;
            public static final double kI = 0.00002;
            public static final double kD = 0.002;
        }

        public static final class Odometry {
          // TODO: ADJUST THESE STANDARD DEVIATIONS
          public static final Matrix<N3, N1> STATE_STANDARD_DEVS = new Matrix<>(Nat.N3(), Nat.N1());
          public static final Matrix<N3, N1> VISION_STANDARD_DEVS = new Matrix<>(Nat.N3(), Nat.N1());
          static {
            STATE_STANDARD_DEVS.set(0, 0, 0.2); // State x position
            STATE_STANDARD_DEVS.set(1, 0, 0.2); // State y position
            STATE_STANDARD_DEVS.set(2, 0, 0.2); // State rotation

            VISION_STANDARD_DEVS.set(0, 0, 30); // Vision x position
            VISION_STANDARD_DEVS.set(1, 0, 30); // Vision y position
            VISION_STANDARD_DEVS.set(2, 0, 30); // Vision rotation
          }

          public static final double MIN_TIME_BETWEEN_LL_UPDATES_MS = 20e-3;
        }
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 5;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = 2 * Math.PI;
    
        public static final double kPXController = 10;
        public static final double kPYController = 10;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

  public static class motorConstants {
    public static final double falconFreeSpeedRPM = 6380.0;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class Arm {

    public static final int shoulderMotorID = 50;

    public static final double shoulderMotorP = 0.1;
    public static final double shoulderMotorI = 0.01;
    public static final double shoulderMotorD = 0;
    public static final double shoulderMotorF = 0.06;
    public static final double shoulderMotorIZone = 5000;
    public static final double shoulderCruiseVelocity = 20000; // Degrees per second
    public static final double shoulderAccelerationFactor = 25000; // Degrees per second squared

    public static final int elbowMotorID = 51;

    public static final double elbowMotorP = 0.1;
    public static final double elbowMotorI = 0.001;
    public static final double elbowMotorD = 0;
    public static final double elbowMotorF = 0.06;
    public static final double elbowMotorIZone = 1000;
    public static final double elbowMaxIntegeralAccumulator = 0;
    public static final double elbowCruiseVelocity = 15000; // sensorUnitsPer100ms 
    public static final double elbowAccelerationFactor = 25000; // sensorUnitsPer100ms per second

    public static final double upperarmLength = 1.033; // Meters
    public static final double LimelightCenterToShoulderPivot = 0.13;//Meters
    public static final double forearmLength = 0.96; // Meters

    public static final double upperarmMass = 0.44639; // Kilograms
    public static final double forearmMass = 0.293; // Kilograms
    public static final double elbowMass = 0.95; // Kilograms

    public static final int shoulderCanCoderID = 1;
    public static final double shoulderAngleActual = 43.9; // Degrees
    public static final double shoulderAngleSensor = 230.713; // Degrees
    public static final double shoulderAngleReverseSoftStop = 38; // Degrees
    public static final double shoulderAngleForwardSoftStop = 130; // Degrees

    public static final int elbowCanCoderID = 2;
    public static final double elbowAngleActualDifference = -64.6; // Degrees
    public static final double elbowAngleSensor = 243.721 ; // Degrees
    public static final double elbowAngleForwardSoftStop = 90; // Degrees
    public static final double elbowAngleReverseSoftStop = -160; // Degrees

    public static final int extraSolenoid = 0;
    public static final int elbowBrakeSolenoid = 1;
    public static final int shoulderBrakeSolenoid = 2;

    public static final double elbowLockThreshold = 1; // degrees
    public static final double shoulderLockThreshold = 1; // degrees

    
    public static final int smoothingFactor = 2; // Arbitrary 1-8
    
    public static final double ManualAdjustMPS = 0.1; //meters per second
    
    public static final double allowableError = 10;
    
    public static final int blinkenPWM_ID = 0;

    public static final double shoulderGearRatio = 54.0/9.0*76.0/20.0*76.0/20.0*40.0/15.0;
    public static final double shoulderDegreesPerMotorTick = 360.0/2048.0 / shoulderGearRatio;

    public static final double elbowGearRatio = 54.0/9.0*76.0/20.0*40.0/15.0;
    public static final double elbowVirtualFourBarRatio = 54.0/26.0;
    public static final double elbowDegreesPerMotorTick = 360.0/2048.0 / Constants.Arm.elbowGearRatio / elbowVirtualFourBarRatio;

    public static final double floorHeight = -0.15;

    public static class SetPoint {
      //Scoring Positions
      public static final PathPoint cubeHighPosition = new PathPoint(new Translation2d(1.5, 1), Rotation2d.fromDegrees(-30), "cubeHighPosition").withControlLengths(0.5, 0.5);
      public static final PathPoint coneHighPosition = new PathPoint(new Translation2d(1.44, 1.3), Rotation2d.fromDegrees(-35), "coneHighPosition").withControlLengths(0.75, .75);
      public static final PathPoint cubeMediumPosition = new PathPoint(new Translation2d(1.067, 0.781), Rotation2d.fromDegrees(-30), "cubeMediumPosition").withControlLengths(0.5, 0.5);
      public static final PathPoint coneMediumPosition = new PathPoint(new Translation2d(1.042, 0.987), Rotation2d.fromDegrees(-30), "coneMediumPosition").withControlLengths(0.5, 0.5);
      public static final PathPoint lowPosition = new PathPoint(new Translation2d(0.7, 0.158), Rotation2d.fromDegrees(-90), "lowPosition").withControlLengths(0.25, 0.25);

      //Intaking Positions
      public static final PathPoint loadingZonePosition = new PathPoint(new Translation2d(0.666, 1.005), Rotation2d.fromDegrees(0), "loadingZonePosition").withControlLengths(0.25, 0.25);

      // private static final Vector2D intakeTweenPosition = new Vector2D(0.59, 0.22);

      public static final PathPoint upIntakePosition = new PathPoint(new Translation2d(0.59, 0.09), Rotation2d.fromDegrees(-90), "upIntakePosition").withControlLengths(0.25, 0.25);
      public static final PathPoint generalIntakePosition = new PathPoint(new Translation2d(0.59, -0.04), Rotation2d.fromDegrees(-90), "generalIntakePosition").withControlLengths(0.25, 0.25);

      //Stowing Positions
      public static final PathPoint stowPosition = new PathPoint(new Translation2d(0.423, 0.324), Rotation2d.fromDegrees(-90), "stowPosition");

      public static final PathPoint startPosition = new PathPoint(new Translation2d(0.27, 0.18), Rotation2d.fromDegrees(-135), "startPosition").withControlLengths(0.25, 0.25);
    }

  }

  public static class Hand {
    public static final int mHandSolenoidFwd = 7;
    public static final int mHandSolenoidRev = 5;

    public static final double handMass = 2.1;

    public static final Utils.Vector2D maxFrameExtension = new Utils.Vector2D((48+15-5) / 39.37, (78-5)/39.37);
  }
}
