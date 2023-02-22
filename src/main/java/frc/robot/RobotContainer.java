
package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Arm.FineAdjust;
import frc.robot.commands.Arm.SetArmPosition;
import frc.robot.commands.Arm.ToggleHand;
import frc.robot.commands.Autos.Balance;
import frc.robot.commands.Hand.SetHand;
import frc.robot.commands.Swerve.Flatten;
import frc.robot.commands.Swerve.TeleopSwerve;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.subsystems.Hand.HandHolding;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final XboxController mDriverController = new XboxController(0);
    private final XboxController mManipController = new XboxController(1);
  
    /* Subsystems */
    private final Swerve s_Swerve = Swerve.getInstance();
    private final Arm s_Arm = Arm.getInstance();
    // private final Hand s_Hand = Hand.getInstance();

    public HandHolding currentHolding = HandHolding.CONE;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                () -> -mDriverController.getLeftY(), 
                () -> -mDriverController.getLeftX(), 
                () -> -mDriverController.getRightX(), 
                () ->  mDriverController.getAButton()
            )
        );

        new JoystickButton(mDriverController, XboxController.Button.kY.value).whileTrue(new Flatten(0.3));
        new JoystickButton(mDriverController, XboxController.Button.kX.value).whileTrue(new Balance());
        new JoystickButton(mDriverController, XboxController.Button.kBack.value).toggleOnTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

        s_Arm.setDefaultCommand(
            new FineAdjust(
                () -> mManipController.getLeftY(),
                () -> mManipController.getRightY()
            )
        );

        new JoystickButton(mManipController, XboxController.Button.kY.value).whileTrue(new ToggleHand());
        
        new JoystickButton(mManipController, XboxController.Button.kLeftBumper.value).toggleOnTrue(new SetArmPosition(Arm.ArmPosition.kHighPositionCone));
        new JoystickButton(mManipController, XboxController.Button.kX.value).whenPressed(new SetArmPosition(Arm.ArmPosition.kMediumPositionCone));
        new JoystickButton(mManipController, XboxController.Button.kRightBumper.value).whenPressed(new SetArmPosition(Arm.ArmPosition.kHighPositionCube));
        new JoystickButton(mManipController, XboxController.Button.kB.value).whenPressed(new SetArmPosition(Arm.ArmPosition.kMediumPositionCube));
        new JoystickButton(mManipController, XboxController.Button.kA.value).whenPressed(new SetArmPosition(Arm.ArmPosition.kStowPosition));
        new JoystickButton(mManipController, XboxController.Button.kLeftStick.value).whenPressed(new SetArmPosition(Arm.ArmPosition.kIntakePosition));

        new JoystickButton(mManipController, XboxController.Button.kLeftBumper.value).toggleOnTrue(new InstantCommand(() -> switchHandHolding()));
        
    }

    public void switchHandHolding() {
        currentHolding = currentHolding == HandHolding.CUBE ? HandHolding.CONE : HandHolding.CUBE;
    }

    public HandHolding getHandHolding() {
        return currentHolding;
    }

}
