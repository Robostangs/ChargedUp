// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Drivetrain.ArcadeDrive;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;


public class RobotContainer {
  private final XboxController mDriverController = new XboxController(0);
  private final XboxController mManipController = new XboxController(1);

  private final Hand mHand = Hand.getInstance();
  private final Arm mArm = Arm.getInstance();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    mArm.setStickPowerSupplier(() -> (mDriverController.getLeftY() * 0.2));
  }

 
  public Command getAutonomousCommand() {
    return null;
  }
}
