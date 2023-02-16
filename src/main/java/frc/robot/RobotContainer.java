package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Autos.balance;
import frc.robot.commands.Drivetrain.Flatten;
import frc.robot.commands.Drivetrain.TeleopSwerve;
import frc.robot.subsystems.*;

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

    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;
  
    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(mDriverController, XboxController.Button.kBack.value);
    private final JoystickButton robotCentric = new JoystickButton(mDriverController, XboxController.Button.kLeftBumper.value);

    /* Subsystems */
    private final Swerve s_Swerve = Swerve.getInstance();
    private final Arm s_Arm = Arm.getInstance();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        
        // if(mDriverController.getAButton()) {
        //     new Flatten(0.3); 
        // } else if(mDriverController.getBButton()) {
        //     new balance();
        // } else {
        //     s_Swerve.setDefaultCommand(
        //         new TeleopSwerve(
        //             () -> -mDriverController.getRawAxis(translationAxis), 
        //             () -> -mDriverController.getRawAxis(strafeAxis), 
        //             () -> -mDriverController.getRawAxis(rotationAxis), 
        //             () -> robotCentric.getAsBoolean()
        //         )
        //     );
        // }
        // Configure the button bindings
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
                () -> -mDriverController.getRawAxis(translationAxis), 
                () -> -mDriverController.getRawAxis(strafeAxis), 
                () -> -mDriverController.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );

        s_Arm.setStickSupplier(() -> mManipController.getRawAxis(translationAxis));
        s_Arm.setStickEnableSupplier(() -> mManipController.getAButton());

        new JoystickButton(mDriverController, XboxController.Button.kBack.value).toggleOnTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        new JoystickButton(mDriverController, XboxController.Button.kB.value).whileTrue(new balance());
        new JoystickButton(mDriverController, XboxController.Button.kA.value).whileTrue(new Flatten(0.3));
    }
}
