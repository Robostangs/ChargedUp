package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.MusicCMD;
import frc.robot.commands.Arm.ArmManager;
import frc.robot.commands.Arm.ArmStupid;
// import frc.robot.commands.Drivetrain.TeleopSwerve;
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


    private TalonFX talonShoulder, talonElbow;
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;
  
    // private final Hand mHand = Hand.getInstance();
    // private final Arm mArm = Arm.getInstance();

  //private final Drivetrain mDrivetrain = Drivetrain.getInstance();
//   private final Vision mVision = Vision.getInstance();

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(mDriverController, XboxController.Button.kBack.value);
    private final JoystickButton robotCentric = new JoystickButton(mDriverController, XboxController.Button.kLeftBumper.value);

    /* Subsystems */
    // private final Swerve s_Swerve = Swerve.getInstance();
    private final Arm s_Arm = Arm.getInstance();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        talonShoulder = new TalonFX(50);
        talonElbow = new TalonFX(51);

        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        // s_Swerve.setDefaultCommand(
        //     new TeleopSwerve(
        //         () -> -mDriverController.getRawAxis(translationAxis), 
        //         () -> -mDriverController.getRawAxis(strafeAxis), 
        //         () -> -mDriverController.getRawAxis(rotationAxis), 
        //         () -> true
        //     )
        // );
        // // Configure the button bindings

            //Music Selector
        Music mMusic = Music.getInstance();
        Arm mArm = new Arm();
        mMusic.setDefaultCommand(new MusicCMD(
            mManipController.getStartButton(),
            "Crab-Rave.chrp",
            false,
            mArm.getTalonFXs()
        )
        );        

        s_Arm.setDefaultCommand(
            new ArmStupid(
                () -> mManipController.getLeftY(),  
                () -> mManipController.getRightY())
        );

        // s_Arm.setShoulderMotorPower(mManipController.getLeftY());
        // s_Arm.setElbowMotorPower(mManipController.getRightY());


        // zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
