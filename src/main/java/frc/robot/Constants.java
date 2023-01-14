// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.revrobotics.SparkMaxPIDController;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class motorConstants {
    public static final double falconFreeSpeedRPM = 6380.0;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class Drivetrain{

    public static final double maxVoltage = 12.0;
    //TODO: add left-right trackWidth , add front-back wheelBase
    public static final double trackWidth = 0.0;
    public static final double wheelBase = 0.0;
    public static final double maxLinearVelocity = motorConstants.falconFreeSpeedRPM / 60.0 *
    SdsModuleConfigurations.MK4I_L1.getDriveReduction() *
    SdsModuleConfigurations.MK4I_L1.getWheelDiameter() * Math.PI;
    public static final double maxAngularVelocity = maxLinearVelocity / Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
    
    public static final SwerveDriveKinematics mKinematics = new SwerveDriveKinematics(
      new Translation2d(trackWidth/2.0, wheelBase/2.0),
      new Translation2d(trackWidth/2.0, -wheelBase/2.0),
      new Translation2d(-trackWidth/2.0, wheelBase/2.0),
      new Translation2d(-trackWidth/2.0, -wheelBase/2.0)
    );

    public static final int frontLeftDriveID = 0;
    public static final int frontLeftSteerID = 1;
    public static final int frontLeftEncoderID = 2;
    public static final double frontLeftEncoderOffset = 0;

    public static final int frontRightDriveID = 3;
    public static final int frontRightSteerID = 4;
    public static final int frontRightEncoderID = 5;
    public static final double frontRightEncoderOffset = 0;

    public static final int backLeftDriveID = 6;
    public static final int backLeftSteerID = 7;
    public static final int backLeftEncoderID = 8;
    public static final double backLeftEncoderOffset = 0;

    public static final int backRightDriveID = 0;
    public static final int backRightSteerID = 0;
    public static final int backRightEncoderID = 0;
    public static final double backRightEncoderOffset = 0;

    public static final TalonFXConfiguration leftFrontMotorConfig = new TalonFXConfiguration();
    static {
      leftFrontMotorConfig.slot0 = new SlotConfiguration();
      leftFrontMotorConfig.slot0.kP = 0;
      leftFrontMotorConfig.slot0.kI = 0;
      leftFrontMotorConfig.slot0.kD = 0;
    }
    
    public static final TalonFXConfiguration leftBackMotorConfig = new TalonFXConfiguration();
    static {
      leftBackMotorConfig.slot0 = new SlotConfiguration();
      leftBackMotorConfig.slot0.kP = 0;
      leftBackMotorConfig.slot0.kI = 0;
      leftBackMotorConfig.slot0.kD = 0;
    }

    public static final TalonFXConfiguration rightFrontMotorConfig = new TalonFXConfiguration();
    static {
      rightFrontMotorConfig.slot0 = new SlotConfiguration();
      rightFrontMotorConfig.slot0.kP = 0;
      rightFrontMotorConfig.slot0.kI = 0;
      rightFrontMotorConfig.slot0.kD = 0;
    }

    public static final TalonFXConfiguration rightBackMotorConfig = new TalonFXConfiguration();
    static {
      rightBackMotorConfig.slot0 = new SlotConfiguration();
      rightBackMotorConfig.slot0.kP = 0;
      rightBackMotorConfig.slot0.kI = 0;
      rightBackMotorConfig.slot0.kD = 0;
    }

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
  }
  }

  public static class Arm {

    public static final int shoulderMotorID = 0;

    public static final int shoulderMotorP = 0;
    public static final int shoulderMotorI = 0;
    public static final int shoulderMotorD = 0;
    public static final int shoulderMotorIZone = 0;
    
    public static final int elbowMotorID = 0;

    public static final int elbowMotorP = 0;
    public static final int elbowMotorI = 0;
    public static final int elbowMotorD = 0;
    public static final int elbowMotorIZone = 0;
    
    public static final double upperarmLength = 0;
    public static final double forearmLength = 0;
  }
}
