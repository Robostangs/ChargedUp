// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.util.Map;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.LoggyThings.LoggyThingManager;
import frc.robot.commands.Swerve.TeleopSwerve;
import frc.robot.commands.Arm.SetArmPosition;
import frc.robot.commands.Lights.LightCMD;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Hand;
import frc.robot.subsystems.Lighting;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Arm.ArmPosition;
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
  public static PowerDistribution mPowerDistribution = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);
  public static SendableChooser<String> chooser;
  public static SendableChooser<Command> test;
  ShuffleboardTab testTab;
  static Boolean testsAdded;
  

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    ctreConfigs = new CTREConfigs();
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    mPowerDistribution.setSwitchableChannel(true);

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

    SmartDashboard.putData("jefy", chooser);
    SmartDashboard.putBoolean("isRed", false);

    CommandScheduler.getInstance().onCommandInitialize((Command c) -> {DataLogManager.log("INITIALIZED: " + c.getName());});
    CommandScheduler.getInstance().onCommandFinish((Command c) -> {DataLogManager.log("FINISHED: " + c.getName());});
    CommandScheduler.getInstance().onCommandInterrupt((Command c) -> {DataLogManager.log("INTERUPTED: " + c.getName());});

    testsAdded = false;
    SmartDashboard.putNumber("Test Speed", Constants.Swerve.testSpeed);
    test = new SendableChooser<Command>();
    Command stop = new InstantCommand();
    stop.setName("Disabled");
    test.setDefaultOption("Disable", stop);
    for (int x = 0; x < PITTest.cmdList.length; x++) {
      PITTest.commandList[x].setName(PITTest.cmdList[x]);
      test.addOption(PITTest.commandList[x].getName(), PITTest.commandList[x]);
      // System.out.println("Added Command: " + PITTest.commandList[x].getName());
    }
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
    m_autonomousCommand = new RobotContainer().getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_robotContainer = new RobotContainer();
    new InstantCommand(() -> {new LightCMD(Lighting.killLights());}).schedule();
    new SetArmPosition(ArmPosition.kStartPosition).schedule();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    Vision.getInstance().getTargetHandX();
    System.out.println(CommandScheduler.getInstance().getDefaultCommand(Swerve.getInstance()));
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().close();
    CommandScheduler.getInstance().enable();
    if (testsAdded == false) {
      testTab = Shuffleboard.getTab("PIT Test");
      testTab.add("Test Select", test).withWidget(BuiltInWidgets.kComboBoxChooser);
      
      testTab.addDouble("Testing Speed",() -> SmartDashboard.getNumber("Test Speed", 0))
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", -1, "max", 1));
      
      testTab.add("Increase Speed", new InstantCommand(() -> {PITTest.speedPlus();})
      .andThen(new InstantCommand(() -> {SmartDashboard.putNumber("Test Speed", Constants.Swerve.testSpeed);})))
      .withWidget(BuiltInWidgets.kCommand);
      
      testTab.add("Decrease Speed", new InstantCommand(() -> {PITTest.speedMinus();})
      .andThen(new InstantCommand(() -> {SmartDashboard.putNumber("Test Speed", Constants.Swerve.testSpeed);})))
      .withWidget(BuiltInWidgets.kCommand);
      testsAdded = true;
      
      testTab.addString("PIT Test Status", () -> SmartDashboard.getString("pitStat", "null"))
      .withWidget(BuiltInWidgets.kTextView);

      SmartDashboard.putString("pitStat", "Initizalizing");
      
      PITTest.init();
    }

    new InstantCommand(() -> {new LightCMD(0.73);}).schedule();
    Shuffleboard.selectTab(testTab.getTitle());
  }
  
  
  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    // PITTest.PDH();
    Command newCMD = test.getSelected();
    SmartDashboard.putString("pitStat", newCMD.getName());
    if (newCMD.getName() != "Translation Test" && newCMD.getName() != "Strafe Test" && newCMD.getName() != "Rotation Test" && newCMD.getName() != "Drivetrain Control") {
      new TeleopSwerve(() -> 0, () -> 0, () -> 0, () -> false, () -> false).withName("Kill Drivetrain").schedule();
    } else if (!CommandScheduler.getInstance().isScheduled(newCMD)) {
      newCMD.schedule();
    }

    // Command x = Swerve.getInstance().getCurrentCommand();
    // Command y = Hand.getInstance().getCurrentCommand();
    // Command z = Arm.getInstance().getCurrentCommand();

    // if (x != null) {
    //   System.out.println(x.getName());
    // } else {
    //   System.out.println(x);
    // }

    // if (y != null) {
    //   System.out.println(y.getName());
    // } else {
    //   System.out.println(y);
    // }

    // if (z != null) {
    //   System.out.println(z.getName());
    // } else {
    //   System.out.println(z);
    // }

    //Seems to work, it will change the command listed in the console based on whatever i am selecting in dashboard
  }

  @Override
  public void testExit() {
    CommandScheduler.getInstance().cancelAll();
    Shuffleboard.selectTab("SmartDashboard");
  }
}