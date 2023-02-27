

package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Vision.LimelightMeasurement;
import frc.robot.commands.Arm.FineAdjust;
import frc.robot.commands.Arm.SetArmPosition;
import frc.robot.commands.Autos.balance;
import frc.robot.commands.Hand.SetGrip;
import frc.robot.commands.Hand.SetLightColor;
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
    private final XboxController mDriverController = new XboxController(0);
    private final XboxController mManipController = new XboxController(1);
    private final CommandXboxController mCommandDriver = new CommandXboxController(0);
    private final CommandXboxController mCommandManip = new CommandXboxController(1);

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
                () ->  mDriverController.getAButton()
            )
        );

        mCommandDriver.y().whileTrue(new Flatten(0.3));
        mCommandDriver.x().whileTrue(new balance());
        mCommandDriver.back().toggleOnTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

        s_Arm.setDefaultCommand(
            new FineAdjust(
                () -> Utils.customDeadzone(-mManipController.getLeftX()),
                () -> Utils.customDeadzone(-mManipController.getLeftY())
            )
        );

        // Upon looking at the documentation whenPressed DOES do something different and it is what we want
        mCommandManip.leftBumper().whileTrue(new SetGrip()); 
        mCommandManip.y().onTrue(new SetArmPosition(ArmPosition.kHighPosition, s_Hand.getHolding()));
        mCommandManip.b().onTrue(new SetArmPosition(ArmPosition.kMediumPosition, s_Hand.getHolding()));
        mCommandManip.a().onTrue(new SetArmPosition(ArmPosition.kLowPosition, s_Hand.getHolding()));
        mCommandManip.x().onTrue(new SetArmPosition(ArmPosition.kIntakePosition, s_Hand.getHolding()));
        mCommandManip.leftStick().onTrue(new SetArmPosition(ArmPosition.kStowPosition, s_Hand.getHolding()));
        mCommandManip.rightBumper().onTrue(new ToggleHolding());
        mCommandManip.rightStick().onTrue(new SetArmPosition(ArmPosition.kLoadingZonePosition, s_Hand.getHolding()));

        mCommandDriver.rightBumper().onTrue(new InstantCommand(() -> {
            Optional<LimelightMeasurement> leftMeasurement = s_Vision.getNewLeftMeasurement();
            if (leftMeasurement.isPresent()) {
                s_Swerve.resetOdometry(leftMeasurement.get().mPose);
            }
        }));

        mCommandDriver.leftBumper().onTrue(new StraightenManager(s_Hand.getHolding()));
        new SetLightColor(56).schedule();

        // mCommandDriver.leftBumper().onTrue(new StraightenManager(s_Hand.getHolding()));
    }

    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        // return new translate(s_Swerve, s_Vision.);
        // TODO
        return null;
    }

}
