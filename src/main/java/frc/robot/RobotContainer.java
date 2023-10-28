
package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.LoggyThings.LoggyPrintCommand;
import frc.robot.autos.autoFromPath;
import frc.robot.autos.charlieAutoGrab;
import frc.robot.commands.Arm.PercentOutput;
import frc.robot.commands.Hand.SetGrip;
import frc.robot.commands.Hand.ToggleHolding;
import frc.robot.commands.Lights.LightReqCMD;
import frc.robot.commands.Arm.ProfiledChangeSetPoint;
import frc.robot.commands.Swerve.GetToPosition;
import frc.robot.commands.Swerve.TeleopSwerve;
import frc.robot.commands.Swerve.balance;
import frc.robot.subsystems.*;
import frc.robot.Constants.Lights;
import frc.robot.Vision.LimelightMeasurement;

public class RobotContainer {
    /* Controllers */

    private boolean lightsCone = true;
    private Spark blinkin = new Spark(Lights.blinkinPWM_ID);

    public static final XboxController mDriverController = new XboxController(0);
  
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
                () ->  mDriverController.getStartButton(),
                () ->  mDriverController.getLeftBumper()
            )
        );

        s_Arm.setDefaultCommand(
            new PercentOutput(
                () -> Utils.customDeadzone(0),
                () -> Utils.customDeadzone(mDriverController.getLeftTriggerAxis()-mDriverController.getRightTriggerAxis())
            )
        );


        new POVButton(mDriverController, 0).whileTrue(new balance());
        new JoystickButton(mDriverController, XboxController.Button.kBack.value).toggleOnTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));                

        new JoystickButton(mDriverController, XboxController.Button.kLeftBumper.value).whileTrue(new SetGrip()); 
        new JoystickButton(mDriverController, XboxController.Button.kY.value).onTrue(ProfiledChangeSetPoint.createWithTimeout(() -> s_Hand.holdingCone?Constants.Arm.SetPoint.coneHighPosition:Constants.Arm.SetPoint.cubeHighPosition));
        new JoystickButton(mDriverController, XboxController.Button.kB.value).onTrue(ProfiledChangeSetPoint.createWithTimeout(() -> s_Hand.holdingCone?Constants.Arm.SetPoint.coneMediumPosition:Constants.Arm.SetPoint.cubeMediumPosition));
        new JoystickButton(mDriverController, XboxController.Button.kA.value).onTrue(ProfiledChangeSetPoint.createWithTimeout(() -> Constants.Arm.SetPoint.lowPosition));
        new JoystickButton(mDriverController, XboxController.Button.kX.value).onTrue(ProfiledChangeSetPoint.createWithTimeout(() -> Constants.Arm.SetPoint.generalIntakePosition));
        new JoystickButton(mDriverController, XboxController.Button.kLeftStick.value).onTrue(ProfiledChangeSetPoint.createWithTimeout(() -> Constants.Arm.SetPoint.stowPosition));
        new JoystickButton(mDriverController, XboxController.Button.kRightStick.value).onTrue(ProfiledChangeSetPoint.createWithTimeout(() -> Constants.Arm.SetPoint.loadingZonePosition));
        new POVButton(mDriverController, 180).onTrue(ProfiledChangeSetPoint.createWithTimeout(() -> Constants.Arm.SetPoint.startPosition));

        new POVButton(mDriverController, 270).onTrue(new ToggleHolding().andThen(new WaitCommand((2))).andThen(()->mDriverController.setRumble(RumbleType.kBothRumble, 0)).handleInterrupt(()->mDriverController.setRumble(RumbleType.kBothRumble, 0)));
        
        new POVButton(mDriverController, 90).onTrue(new ConditionalCommand(new InstantCommand(() -> blinkin.set(Lights.kConeBlink)), new InstantCommand(() -> blinkin.set(Lights.kCubeBlink)), () -> lightsCone).andThen(new InstantCommand(() -> lightsCone = !lightsCone)));
        
    }

    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new autoFromPath();
    }
}
