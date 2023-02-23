package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.autos.rotation;
import frc.robot.commands.AestheticsCMD.LightReqCMD;
import frc.robot.commands.AestheticsCMD.MusicCMD;
import frc.robot.commands.Arm.FineAdjust;
import frc.robot.commands.Arm.SetArmPosition;
import frc.robot.commands.Hand.ToggleHolding;
import frc.robot.commands.Swerve.TeleopSwerve;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Arm.ArmPosition;

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
    private final Hand s_Hand = Hand.getInstance();
    // private final Hand s_Hand = Hand.getInstance();

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
                () -> -mDriverController.getLeftX(), 
                () -> -mDriverController.getLeftY(), 
                () -> -mDriverController.getRightX(), 
                () ->  mDriverController.getBackButton()
            )
        );
        
        SmartDashboard.putData("Play Music", new MusicCMD());

        // mDriverController.getYButton().whenPressed(new Flatten(0.3));
        // mDriverController.getXButton().whenPressed(new Balance());
        // mDriverController.getBackButton().whenPressed(new InstantCommand(() -> s_Swerve.zeroGyro()));

        s_Arm.setDefaultCommand(
            new FineAdjust(
                () -> mManipController.getLeftY(),
                () -> mManipController.getRightY()
            )
        );

        // mManipController.getLeft().whileHeld(new Hand(0.5));

        // new JoystickButton(mManipController, XBoxController.class).whileTrue(new SetHand());

        new JoystickButton(mManipController, XboxController.Button.kLeftBumper.value).whileTrue(new ToggleHolding());
        new JoystickButton(mManipController, XboxController.Button.kY.value).whileTrue(new SetArmPosition(ArmPosition.kHighPosition, s_Hand.getHolding()));
        new JoystickButton(mManipController, XboxController.Button.kB.value).whileTrue(new SetArmPosition(ArmPosition.kMediumPosition, s_Hand.getHolding()));
        new JoystickButton(mManipController, XboxController.Button.kA.value).whileTrue(new SetArmPosition(ArmPosition.kLowPosition, s_Hand.getHolding()));
        new JoystickButton(mManipController, XboxController.Button.kX.value).whileTrue(new SetArmPosition(ArmPosition.kIntakePosition, s_Hand.getHolding()));
        
        new JoystickButton(mManipController, XboxController.Button.kRightBumper.value).whileTrue(new SetArmPosition(ArmPosition.kLoadingZonePosition, s_Hand.getHolding()));
    
        new POVButton(mManipController, 270).toggleOnTrue(new LightReqCMD(270));
        new POVButton(mManipController, 180).toggleOnTrue(new LightReqCMD(180));
    }

    /** Autonomous Commands that run in the first 15 seconds of the game. */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new rotation(s_Swerve, 30);
    }

}
