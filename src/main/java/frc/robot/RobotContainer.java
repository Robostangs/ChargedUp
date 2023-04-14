
package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.autoFromPath;
import frc.robot.autos.charlieAutoGrab;
import frc.robot.commands.Arm.PercentOutput;
import frc.robot.commands.Hand.SetGrip;
import frc.robot.commands.Lights.LightCMD;
import frc.robot.commands.Lights.LightReqCMD;
import frc.robot.commands.Arm.ProfiledChangeSetPoint;
import frc.robot.commands.Swerve.GetToPosition;
import frc.robot.commands.Swerve.TeleopSwerve;
import frc.robot.commands.Swerve.balance;
import frc.robot.subsystems.*;
import frc.LoggyThings.LoggyPrintCommand;
import frc.robot.Vision.LimelightMeasurement;

public class RobotContainer {
    /* Controllers */

    public static final XboxController mDriverController = new XboxController(0);
    public static final XboxController mManipController = new XboxController(1);
  
    /* Subsystems */
    private final Swerve s_Swerve = Swerve.getInstance();
    private final Arm s_Arm = Arm.getInstance();
    private final Hand s_Hand = Hand.getInstance();
    private final Vision s_Vision = Vision.getInstance();
    
    public RobotContainer() {
        configureButtonBindings();
    }

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

        s_Arm.setDefaultCommand(
            new PercentOutput(
                () -> Utils.customDeadzone(-mManipController.getLeftY()),
                () -> Utils.customDeadzone(-mManipController.getRightY())
            )
        );

        // new JoystickButton(mDriverController, XboxController.Button.kY.value).whileTrue(new Flatten(0.3));
        new JoystickButton(mDriverController, XboxController.Button.kX.value).whileTrue(new balance());
        new JoystickButton(mDriverController, XboxController.Button.kBack.value).toggleOnTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        new JoystickButton(mDriverController, XboxController.Button.kY.value).toggleOnTrue(new InstantCommand(() -> s_Swerve.lockPosition()));
        // new JoystickButton(mDriverController, XboxController.Button.kA.value).onTrue(new rotation(-s_Vision.getDrivetrainAngle()));
        // new JoystickButton(mDriverController, XboxController.Button.kB.value).whenPressed(new Rotation(-10));
        // new JoystickButton(mDriverController, XboxController.Button.kLeftBumper.value).whenPressed(new StraightenManager(s_Hand.getHolding()));
        // new JoystickButton(mDriverController, XboxController.Button.kLeftBumper.value).whenPressed(new StraightenManager(s_Hand.getHolding()));
        // new POVButton(mDriverController, 90).onTrue(new charlieAutoGrab());
        // new POVButton(mDriverController, 270).onTrue(new GetToPosition());
        // new JoystickButton(mDriverController, XboxController.Button.kB.value).whenPressed(new Rotation(-10));
        // new JoystickButton(mDriverController, XboxController.Button.kLeftBumper.value).whenPressed(new StraightenManager(s_Hand.getHolding()));
        // new JoystickButton(mDriverController, XboxController.Button.kLeftBumper.value).whenPressed(new StraightenManager(s_Hand.getHolding()));
        new JoystickButton(mDriverController, XboxController.Button.kRightBumper.value).onTrue(
            new InstantCommand(() -> {
                Optional<LimelightMeasurement> leftMeasurement = s_Vision.getNewLeftMeasurement();
                Optional<LimelightMeasurement> rightMeasurement = s_Vision.getNewRightMeasurement();
                if (leftMeasurement.isPresent()) {s_Swerve.resetOdometry(leftMeasurement.get().mPose);}
                else if(rightMeasurement.isPresent()) {s_Swerve.resetOdometry(rightMeasurement.get().mPose);}})
        );        

        new JoystickButton(mManipController, XboxController.Button.kLeftBumper.value).whileTrue(new SetGrip()); 
        new JoystickButton(mManipController, XboxController.Button.kY.value).onTrue(ProfiledChangeSetPoint.createWithTimeout(() -> s_Hand.holdingCone?Constants.Arm.SetPoint.coneHighPosition:Constants.Arm.SetPoint.cubeHighPosition));
        new JoystickButton(mManipController, XboxController.Button.kB.value).onTrue(ProfiledChangeSetPoint.createWithTimeout(() -> s_Hand.holdingCone?Constants.Arm.SetPoint.coneMediumPosition:Constants.Arm.SetPoint.cubeMediumPosition));
        new JoystickButton(mManipController, XboxController.Button.kA.value).onTrue(ProfiledChangeSetPoint.createWithTimeout(() -> Constants.Arm.SetPoint.lowPosition));
        new JoystickButton(mManipController, XboxController.Button.kX.value).onTrue(ProfiledChangeSetPoint.createWithTimeout(() -> Constants.Arm.SetPoint.generalIntakePosition));
        new JoystickButton(mManipController, XboxController.Button.kLeftStick.value).onTrue(ProfiledChangeSetPoint.createWithTimeout(() -> Constants.Arm.SetPoint.stowPosition));
        new JoystickButton(mManipController, XboxController.Button.kRightStick.value).onTrue(ProfiledChangeSetPoint.createWithTimeout(() -> Constants.Arm.SetPoint.loadingZonePosition));
        new JoystickButton(mManipController, XboxController.Button.kBack.value).onTrue(ProfiledChangeSetPoint.createWithTimeout(() -> Constants.Arm.SetPoint.startPosition));
        Trigger leftTrigger = new JoystickButton(mManipController, XboxController.Axis.kLeftTrigger.value);
        
        new Trigger(() -> mManipController.getLeftTriggerAxis() > 0.5)
            .whileTrue(new InstantCommand(() -> s_Arm.resetLash())
            .alongWith(new LoggyPrintCommand(leftTrigger))
        );

        new POVButton(mDriverController, 90).onTrue(s_Swerve.returnHome.returnHome());

        // new POVButton(mManipController, 270).onTrue(new ToggleHolding().andThen(new WaitCommand((2))).andThen(()->mManipController.setRumble(RumbleType.kBothRumble, 0)).handleInterrupt(()->mManipController.setRumble(RumbleType.kBothRumble, 0)));

        new POVButton(mManipController, 90).onTrue(new LightReqCMD()).debounce(Constants.OperatorConstants.kDebounce);
        new Trigger(() -> Lighting.timer.hasElapsed(Constants.Lights.blinkTime)).onTrue(new InstantCommand(() -> new LightCMD(Lighting.PWMVal).schedule()));

        // new POVButton(mManipController, 270).onTrue(new SetHolding(false).andThen(new WaitCommand((2))).andThen(()->mManipController.setRumble(RumbleType.kBothRumble, 0)).handleInterrupt(()->mManipController.setRumble(RumbleType.kBothRumble, 0)));
        new POVButton(mManipController, 180).onTrue(ProfiledChangeSetPoint.createWithTimeout(() -> Constants.Arm.SetPoint.upIntakePosition));

        new POVButton(mDriverController, 270).onTrue(new GetToPosition());
        new POVButton(mDriverController, 90).onTrue(charlieAutoGrab.getCommand());
    }

    public Command getAutonomousCommand() {
        return new autoFromPath();
    }
}
