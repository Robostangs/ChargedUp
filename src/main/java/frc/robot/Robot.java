// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.LoggyThings.LoggyPrintCommand;
import frc.LoggyThings.LoggyThingManager;
import frc.robot.Constants.Lights;
import frc.robot.autos.autoFromPath;
import frc.robot.autos.pathPlannerChooser;
import frc.robot.commands.Arm.ChangeSetPoint;
import frc.robot.commands.Arm.ProfiledChangeSetPoint;
import frc.robot.commands.Hand.SetGrip;
import frc.robot.commands.Lights.LightCMD;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Hand;
import frc.robot.subsystems.Swerve;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    public static CTREConfigs ctreConfigs;
    public pathPlannerChooser pathPlanner;
    private Command m_autonomousCommand;
    public SendableChooser<String> autonChooser;
    public static SendableChooser<String> chooser;

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        ctreConfigs = new CTREConfigs();
        new RobotContainer();

        chooser = new SendableChooser<String>();
        chooser.setDefaultOption("Nothing", "Nothing");
        chooser.addOption("BlueBalanceLeft", "BlueBalanceLeft.wpilib.json");
        chooser.addOption("BlueBalanceRight", "BlueBalanceRight.wpilib.json");
        chooser.addOption("BlueCenterBalance", "BlueCenterBalance.wpilib.json");
        chooser.addOption("BlueCenterStay", "BlueCenterStay.wpilib.json");
        chooser.addOption("BlueLeaveLeft", "BlueLeaveLeft.wpilib.json");
        chooser.addOption("BlueLeaveRight", "BlueLeaveRight.wpilib.json");
        chooser.addOption("BlueCenterLeftStraight", "BlueCenterLeftStraight.wpilib.json");
        chooser.addOption("BlueCenterRightStraight", "BlueCenterRightStraight.wpilib.json");

        chooser.addOption("RedBalanceLeft", "RedBalanceLeft.wpilib.json");
        chooser.addOption("RedBalanceRight", "RedBalanceRight.wpilib.json");
        chooser.addOption("RedCenterBalance", "RedCenterBalance.wpilib.json");
        chooser.addOption("RedCenterStay", "RedCenterStay.wpilib.json");
        chooser.addOption("RedLeaveLeft", "RedLeaveLeft.wpilib.json");
        chooser.addOption("RedLeaveRight", "RedLeaveRight.wpilib.json");
        chooser.addOption("RedCenterLeftStraight", "RedCenterLeftStraight.wpilib.json");
        chooser.addOption("RedCenterRightStraight", "RedCenterRightStraight.wpilib.json");

        chooser.addOption("DOUBLE AUTO", "tripleAuto");

        SmartDashboard.putData("jefy", chooser);
        SmartDashboard.putBoolean("isRed", false);

        SmartDashboard.putBoolean("Open Side?", true);

        DriverStation.silenceJoystickConnectionWarning(true);

        // autonChooser = new SendableChooser<>();
        // autonChooser.setDefaultOption("Nothing", "null");
        // autonChooser.addOption("Place and Charge Path", "path1");
        /* TODO: Add Auton Options, talk to @mwmcclure7 (Matthew) */
        // SmartDashboard.putData("Autonomous Command", autonChooser);

        CommandScheduler.getInstance().onCommandInitialize((Command c) -> {
            DataLogManager.log("INITIALIZED: " + c.getName());
        });
        CommandScheduler.getInstance().onCommandFinish((Command c) -> {
            DataLogManager.log("FINISHED: " + c.getName());
        });
        CommandScheduler.getInstance().onCommandInterrupt((Command c) -> {
            DataLogManager.log("INTERUPTED: " + c.getName());
        });

        new WaitCommand(1).andThen(new InstantCommand(() -> Arm.getInstance().resetLash()).ignoringDisable(true))
                .andThen(new LoggyPrintCommand("Robot.java reset lash")).ignoringDisable(true).schedule();
    }

    @Override
    public void robotPeriodic() {
        SmartDashboard.putString("path", chooser.getSelected());
        LoggyThingManager.getInstance().periodic();
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
        new LightCMD(Lights.kRobostangs).schedule();

        // m_autonomousCommand = new InstantCommand(() -> Arm.getInstance().resetLash())
        //         .andThen(ProfiledChangeSetPoint.createWithTimeout(() -> Constants.Arm.SetPoint.coneHighPosition))
        //         .andThen(() -> new SetGrip())
        //         .andThen(new WaitCommand(0.5))
        //         .andThen(() -> new SetGrip())
        //         .andThen(ProfiledChangeSetPoint.createWithTimeout(() -> Constants.Arm.SetPoint.stowPosition))
        //         .andThen(new pathPlannerChooser(autonChooser.getSelected()).generateTrajectory());

        m_autonomousCommand = new autoFromPath();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousExit() {
        Swerve.getInstance().getField().getObject(Constants.AutoConstants.kFieldObjectName).close();
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        new LightCMD(Lights.kFireTwinkle).schedule();

        /* Resets swerve odemetry to 180 degrees off rip */
        new InstantCommand(() -> Swerve.getInstance().setGyro(180)).withName("Reset gyro 180°").schedule();
        // new InstantCommand(() -> Swerve.getInstance()
        //         .resetOdometry(new Pose2d(Swerve.getInstance().getPose().getTranslation(),
        //                 Swerve.getInstance().getPose().getRotation().rotateBy(new Rotation2d(180)))))
        //         .alongWith(new InstantCommand(() -> Swerve.getInstance().zeroGyro())).withName("Reset gyro 180° ").schedule();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void simulationInit() {
        // final Joystick xDrive = new Joystick(3);
        // final Swerve s_Swerve = Swerve.getInstance();
        // final Arm s_Arm = Arm.getInstance();
        // final Hand s_Hand = Hand.getInstance();
        // s_Swerve.removeDefaultCommand();
        // s_Arm.removeDefaultCommand();
        // s_Hand.removeDefaultCommand();
        // s_Swerve.setDefaultCommand(
        // new TeleopSwerve(
        // () -> -xDrive.getRawAxis(1),
        // () -> -xDrive.getRawAxis(0),
        // () -> -xDrive.getRawAxis(2),
        // () -> xDrive.getRawButton(0),
        // () -> xDrive.getRawButton(1)
        // )
        // );

        // s_Arm.setDefaultCommand(
        // new PercentOutput(
        // () -> Utils.customDeadzone(-xDrive.getRawAxis(3)),
        // () -> Utils.customDeadzone(-xDrive.getRawAxis(4))
        // )
        // );
    }

    @Override
    public void simulationPeriodic() {
    }
}
