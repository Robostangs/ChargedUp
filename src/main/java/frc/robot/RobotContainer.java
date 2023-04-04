package frc.robot;

import java.time.Instant;
import java.util.Optional;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.LoggyThings.LoggyPrintCommand;
import frc.robot.autos.TripleAuto;
import frc.robot.autos.autoFromPath;
import frc.robot.autos.basicTranslate;
import frc.robot.autos.charlieAutoDriveToCube;
import frc.robot.autos.charlieAutoGrab;
import frc.robot.autos.rotation;
import frc.robot.autos.translate;
import frc.robot.autos.translatePp;
import frc.robot.commands.AestheticsCMD.LightReqCMD;
import frc.robot.commands.Arm.IntakingManager;
import frc.robot.commands.Arm.PercentOutput;
import frc.robot.commands.Arm.ProfiledChangeSetPoint;
import frc.robot.commands.Hand.SetGrip;
import frc.robot.commands.Hand.SetHolding;
import frc.robot.commands.Hand.ToggleHolding;
import frc.robot.commands.Swerve.GetToPosition;
import frc.robot.commands.Swerve.TeleopSwerve;
import frc.robot.commands.Swerve.balance;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Aesthetics.Lighting;
import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.Utils.Vector3D;
import frc.robot.Vision.LimelightMeasurement;
import frc.robot.autos.translatePp;

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

        // new JoystickButton(mDriverController, XboxController.Button.kY.value).whileTrue(new Flatten(0.3));
        new JoystickButton(mDriverController, XboxController.Button.kX.value).whileTrue(new balance());
        new JoystickButton(mDriverController, XboxController.Button.kBack.value).toggleOnTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        new JoystickButton(mDriverController, XboxController.Button.kY.value).toggleOnTrue(new InstantCommand(() -> s_Swerve.lockPosition()));

        // s_Arm.setDefaultCommand(
        //     new FineAdjust(
        //         () -> Utils.customDeadzone(-mManipController.getLeftX()),
        //         () -> Utils.customDeadzone(-mManipController.getLeftY())
        //     )
        // );
        
        s_Arm.setDefaultCommand(
            new PercentOutput(
                () -> Utils.customDeadzone(-mManipController.getLeftY()),
                () -> Utils.customDeadzone(-mManipController.getRightY())
            )
        );

        //Seems like a waste of everyones time
        // Trigger clawGripToggle = new JoystickButton(mManipController, XboxController.Button.kLeftBumper.value);
        // clawGripToggle.whileTrue(new SetGrip());
        
        new JoystickButton(mManipController, XboxController.Button.kLeftBumper.value).whileTrue(new SetGrip()); 
        new JoystickButton(mManipController, XboxController.Button.kY.value).onTrue(ProfiledChangeSetPoint.createWithTimeout(() -> s_Hand.holdingCone?Constants.Arm.SetPoint.coneHighPosition:Constants.Arm.SetPoint.cubeHighPosition));
        new JoystickButton(mManipController, XboxController.Button.kB.value).onTrue(ProfiledChangeSetPoint.createWithTimeout(() -> s_Hand.holdingCone?Constants.Arm.SetPoint.coneMediumPosition:Constants.Arm.SetPoint.cubeMediumPosition));
        new JoystickButton(mManipController, XboxController.Button.kA.value).onTrue(ProfiledChangeSetPoint.createWithTimeout(() -> Constants.Arm.SetPoint.lowPosition));
        new JoystickButton(mManipController, XboxController.Button.kX.value).onTrue(ProfiledChangeSetPoint.createWithTimeout(() -> Constants.Arm.SetPoint.generalIntakePosition));
        new JoystickButton(mManipController, XboxController.Button.kLeftBumper.value).onTrue(ProfiledChangeSetPoint.createWithTimeout(() -> Constants.Arm.SetPoint.stowPosition));
        new JoystickButton(mManipController, XboxController.Button.kRightBumper.value).onTrue(ProfiledChangeSetPoint.createWithTimeout(() -> Constants.Arm.SetPoint.loadingZonePosition));
        new JoystickButton(mManipController, XboxController.Button.kBack.value).onTrue(ProfiledChangeSetPoint.createWithTimeout(() -> Constants.Arm.SetPoint.startPosition));

        Trigger leftTrigger = new JoystickButton(mManipController, XboxController.Axis.kLeftTrigger.value);
        new Trigger(() -> mManipController.getLeftTriggerAxis() > 0.5)
            .whileTrue(new InstantCommand(() -> s_Arm.resetLash())
            .alongWith(new LoggyPrintCommand(leftTrigger)));
       
            
        new JoystickButton(mDriverController, XboxController.Button.kRightBumper.value).whenPressed(() -> {
            Optional<LimelightMeasurement> leftMeasurement = s_Vision.getNewLeftMeasurement();
            Optional<LimelightMeasurement> rightMeasurement = s_Vision.getNewRightMeasurement();
            if (leftMeasurement.isPresent()) {
                s_Swerve.resetOdometry(leftMeasurement.get().mPose);
            } else if(rightMeasurement.isPresent()) {
                s_Swerve.resetOdometry(rightMeasurement.get().mPose);
            }
        });

        // new JoystickButton(mDriverController, XboxController.Button.kA.value).onTrue(new rotation(-s_Vision.getDrivetrainAngle()));
        new POVButton(mManipController, 270).onTrue(new ToggleHolding().andThen(new WaitCommand((2))).andThen(()->mManipController.setRumble(RumbleType.kBothRumble, 0)).handleInterrupt(()->mManipController.setRumble(RumbleType.kBothRumble, 0)));
        new POVButton(mManipController, 90).onTrue(new ConditionalCommand(new LightReqCMD(90), new LightReqCMD(270), ()->LightReqCMD.lastAngle>90));
        // new POVButton(mManipController, 270).onTrue(new SetHolding(false).andThen(new WaitCommand((2))).andThen(()->mManipController.setRumble(RumbleType.kBothRumble, 0)).handleInterrupt(()->mManipController.setRumble(RumbleType.kBothRumble, 0)));
        new POVButton(mManipController, 180).onTrue(ProfiledChangeSetPoint.createWithTimeout(() -> Constants.Arm.SetPoint.upIntakePosition));

        new POVButton(mDriverController, 270).onTrue(new GetToPosition());
        new POVButton(mDriverController, 90).onTrue(new charlieAutoGrab());
        // new POVButton(mDriverController, 270).onTrue();
        // new JoystickButton(mDriverController, XboxController.Button.kB.value).whenPressed(new Rotation(-10));

        // new JoystickButton(mDriverController, XboxController.Button.kLeftBumper.value).whenPressed(new StraightenManager(s_Hand.getHolding()));
        // new SetLightColor(56).schedule();

        // new JoystickButton(mDriverController, XboxController.Button.kLeftBumper.value).whenPressed(new StraightenManager(s_Hand.getHolding()));
    }

    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new autoFromPath();
    }

    //tbd if needed for the override in changeSetPoint    
    // public XboxController getManipControllerInstance() {
    //     if(instance == null) {
    //         instance = mManipController;
    //     }
    //     return instance;
    // }
}
