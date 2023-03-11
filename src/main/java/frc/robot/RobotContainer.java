

package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.exampleAuto;
import frc.robot.Vision.LimelightMeasurement;
import frc.robot.autos.rotation;
import frc.robot.autos.translate;
import frc.robot.autos.autoFromPath;
import frc.robot.commands.AestheticsCMD.LightCMD;
import frc.robot.commands.AestheticsCMD.LightReqCMD;
import frc.robot.commands.Arm.FineAdjust;
// import frc.robot.commands.Arm.IntakingManager;
import frc.robot.commands.Arm.SetArmPosition;
import frc.robot.commands.Autos.balance;
import frc.robot.commands.Hand.SetGrip;
import frc.robot.commands.Hand.SetHolding;
import frc.robot.commands.Hand.ToggleGrip;
import frc.robot.commands.Hand.ToggleHolding;
import frc.robot.commands.Swerve.Flatten;
import frc.robot.commands.Swerve.StraightenManager;
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
    public final static XboxController mDriverController = new XboxController(0);
    private final XboxController mManipController = new XboxController(1);
  
    /* Subsystems */
    public final static Swerve s_Swerve = Swerve.getInstance();
    private final Arm s_Arm = Arm.getInstance();
    private final Hand s_Hand = Hand.getInstance();
    private final Vision s_Vision = Vision.getInstance();

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
                () ->  mDriverController.getAButton(),
                () ->  mDriverController.getLeftBumper()
            )
        );

        new JoystickButton(mDriverController, XboxController.Button.kY.value).whileTrue(new Flatten(0.3));
        new JoystickButton(mDriverController, XboxController.Button.kX.value).whileTrue(new balance());
        new JoystickButton(mDriverController, XboxController.Button.kBack.value).toggleOnTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

        s_Arm.setDefaultCommand(
            new FineAdjust(
                () -> Utils.customDeadzone(-mManipController.getLeftX()),
                () -> Utils.customDeadzone(-mManipController.getLeftY())
            )
        );

        //Seems like a waste of everyones time
        Trigger clawGripToggle = new JoystickButton(mManipController, XboxController.Button.kLeftBumper.value);
        clawGripToggle.whileTrue(new SetGrip());
        
        new JoystickButton(mManipController, XboxController.Button.kLeftBumper.value).whileTrue(new SetGrip()); 
        new JoystickButton(mManipController, XboxController.Button.kY.value).whenPressed(new SetArmPosition(ArmPosition.kHighPosition, s_Hand.getHolding()));
        new JoystickButton(mManipController, XboxController.Button.kB.value).whenPressed(new SetArmPosition(ArmPosition.kMediumPosition, s_Hand.getHolding()));
        new JoystickButton(mManipController, XboxController.Button.kA.value).whenPressed(new SetArmPosition(ArmPosition.kLowPosition, s_Hand.getHolding()));
        new JoystickButton(mManipController, XboxController.Button.kX.value).whenPressed(new SetArmPosition(ArmPosition.kIntakePositionGeneral, s_Hand.getHolding()));
        new JoystickButton(mManipController, XboxController.Button.kLeftStick.value).whenPressed(new SetArmPosition(ArmPosition.kStowPosition, s_Hand.getHolding()));
        new JoystickButton(mManipController, XboxController.Button.kRightBumper.value).whenPressed(new SetHolding());
        new JoystickButton(mManipController, XboxController.Button.kRightStick.value).whenPressed(new SetArmPosition(ArmPosition.kLoadingZonePosition, s_Hand.getHolding()));

        new JoystickButton(mDriverController, XboxController.Button.kRightBumper.value).whenPressed(() -> {
            Optional<LimelightMeasurement> leftMeasurement = s_Vision.getNewLeftMeasurement();
            Optional<LimelightMeasurement> rightMeasurement = s_Vision.getNewRightMeasurement();
            if (leftMeasurement.isPresent()) {
                s_Swerve.resetOdometry(leftMeasurement.get().mPose);
            } else if(rightMeasurement.isPresent()) {
                s_Swerve.resetOdometry(rightMeasurement.get().mPose);
            }
        });

        new JoystickButton(mDriverController, XboxController.Button.kA.value).whenPressed(new rotation(-s_Vision.getDrivetrainAngle()));
        new POVButton(mManipController, 90).onTrue(new LightReqCMD(90));
        new POVButton(mManipController, 270).onTrue(new LightReqCMD(270));
        new POVButton(mManipController, 180).whenPressed(new SetArmPosition(ArmPosition.kIntakePositionUp, s_Hand.getHolding()));
    }

    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        // return new translate(s_Swerve, s_Vision.);
        // TODO
        return new autoFromPath();
    }
}
