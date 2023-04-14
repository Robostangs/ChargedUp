// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.LoggyThings.LoggyPrintCommand;
import frc.LoggyThings.LoggyThingManager;
import frc.robot.autos.TripleAuto;
import frc.robot.subsystems.Arm;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static CTREConfigs ctreConfigs;

  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  //public static PowerDistribution mPowerDistribution = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);
  public static SendableChooser<String> chooser;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    ctreConfigs = new CTREConfigs();
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    //mPowerDistribution.setSwitchableChannel(true);

    //SmartDashboard.putData("PDH", mPowerDistribution);

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
    chooser.addOption("UP AND OVER", "upandover");

    SmartDashboard.putData("jefy", chooser);
    SmartDashboard.putBoolean("isRed", false);

    SmartDashboard.putBoolean("Open Side?", true);

    CommandScheduler.getInstance().onCommandInitialize((Command c) -> {DataLogManager.log("INITIALIZED: " + c.getName());});
    CommandScheduler.getInstance().onCommandFinish((Command c) -> {DataLogManager.log("FINISHED: " + c.getName());});
    CommandScheduler.getInstance().onCommandInterrupt((Command c) -> {DataLogManager.log("INTERUPTED: " + c.getName());});
   
    new WaitCommand(1).andThen(new InstantCommand(()->Arm.getInstance().resetLash()).ignoringDisable(true)).andThen(new LoggyPrintCommand("Robot.java reset lash")).ignoringDisable(true).schedule();
    // double startTime=System.nanoTime();
    // for(int i=0;i<10;i++)
    //   new ArmTrajectoryPlanner(new PathPoint(new Translation2d(0.2,0.4), Rotation2d.fromDegrees(90)).withControlLengths(0.25, 0.25), new PathPoint(new Translation2d(1.44, 1.3), Rotation2d.fromDegrees(0)).withControlLengths(0.5, 0.5), 7, 7, 2).plan();
    //   System.out.println(new Double(System.nanoTime()-startTime)/10000000);
  }

  /**
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    SmartDashboard.putString("path", chooser.getSelected());
    LoggyThingManager.getInstance().periodic();
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    // new LightCMD(Lights.kRobostangs).schedule();
    
     m_autonomousCommand = new InstantCommand(() -> Arm.getInstance().resetLash()).andThen(m_robotContainer.getAutonomousCommand());

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // new LightCMD(Lights.kFireTwinkle).schedule();
    //ArmTrajectoryPlannerTest.main(null);

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }


    // ProfiledChangeSetPoint.createWithTimeout(
    //                                             new PathPoint(new Translation2d(1.44, 1.3), Rotation2d.fromDegrees(180)).withControlLengths(0.5, 0.5),
    //                                             new PathPoint(new Translation2d(0.27, 0.18), Rotation2d.fromDegrees(180+45)).withControlLengths(0.25, 0.25))
    //                                     .schedule();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //Vision.getInstance().calculateAndPrintGamePiecePosition();

  }

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {
    DriverStation.silenceJoystickConnectionWarning(true);
  //   final Joystick xDrive = new Joystick(3);
  //   final Swerve s_Swerve = Swerve.getInstance();
  //   final Arm s_Arm = Arm.getInstance();
  //   final Hand s_Hand = Hand.getInstance();
  //   s_Swerve.removeDefaultCommand();
  //   s_Arm.removeDefaultCommand();
  //   s_Hand.removeDefaultCommand();
  //   s_Swerve.setDefaultCommand(
  //     new TeleopSwerve(
  //         () -> -xDrive.getRawAxis(1), 
  //         () -> -xDrive.getRawAxis(0), 
  //         () -> -xDrive.getRawAxis(2), 
  //         () ->  xDrive.getRawButton(0),
  //         () ->  xDrive.getRawButton(1)
  //     )
  // );

  // s_Arm.setDefaultCommand(
  //   new PercentOutput(
  //       () -> Utils.customDeadzone(-xDrive.getRawAxis(3)),
  //       () -> Utils.customDeadzone(-xDrive.getRawAxis(4))
  //   )
// );
  }

  @Override
  public void simulationPeriodic() {}
}
