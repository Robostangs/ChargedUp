package frc.robot.Test;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Hand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.Swerve.Mod0;
import frc.robot.Constants.Swerve.Mod1;
import frc.robot.Constants.Swerve.Mod2;
import frc.robot.Constants.Swerve.Mod3;
import frc.robot.commands.AestheticsCMD.LightCMD;
import frc.robot.commands.Arm.PercentOutput;
import frc.robot.commands.Arm.SetArmPosition;
import frc.robot.commands.Hand.ToggleGrip;
import frc.robot.commands.Swerve.TeleopSwerve;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Aesthetics.Lighting;

public class PITTest2 extends CommandBase {
    XboxController xDrive = new RobotContainer().mDriverController;
    double testSpeed = frc.robot.Constants.Swerve.testSpeed;
    PowerDistribution pdp = Robot.mPowerDistribution;

    BooleanSupplier noBoolean = () -> false;
    DoubleSupplier noSpeed = () -> 0;
    
    Command 
        translationTest, strafeTest, rotationTest,
        toggleGrip,
        setIntakePOS, setStowPOS, rawPower;
    Command[] commandList = {
        /* Swerve testing */
        translationTest = new TeleopSwerve(() -> testSpeed, () -> 0, () -> 0, () -> false, () -> false),
        strafeTest = new TeleopSwerve(() -> 0, () -> testSpeed, () -> 0, () -> false, () -> false),
        rotationTest = new TeleopSwerve(() -> 0, () -> 0, () -> testSpeed, () -> false, () -> false),
        /* Arm Testing */
        toggleGrip = new ToggleGrip(),
        /* Hand Testing */
        setIntakePOS = new SetArmPosition(ArmPosition.kIntakePositionGeneral, false),
        setStowPOS = new SetArmPosition(ArmPosition.kStowPosition, true),
        rawPower = new PercentOutput(() -> xDrive.getRightY(), () -> xDrive.getLeftY())
    };

    SendableChooser<Command> chooser = new SendableChooser<Command>();
    ArrayList<Command> testedCommands = new ArrayList<Command>();
    int currCommand = 0;
    Command lastRun = null;
    Command init = new WaitCommand(1);
    static boolean ran = false;

    public PITTest2() {
        this.addRequirements(Swerve.getInstance(), Hand.getInstance(), Arm.getInstance(), Lighting.getInstance());
        setName("PIT Test");
        Shuffleboard.getTab("Pit Test");
        Shuffleboard.selectTab("Pit Test");
        System.out.println("PITTEST");

        chooser.setDefaultOption("Default (Wait)", init);
        for (Command cmd : commandList) {
            chooser.addOption(cmd.getName(), cmd);
        }
    }

    public void execute() {
        if (chooser.getSelected().isFinished()) {
            if (chooser.getSelected() != lastRun) {
                init.andThen(chooser.getSelected()).schedule();
            } else {
                chooser.getSelected().schedule();
            }
            lastRun = chooser.getSelected();
        }
    }

    public static boolean didRun() {
        return ran;
    }

    public String updateBoard() {
        return chooser.getSelected().getName();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Battery Voltage", () -> RobotController.getBatteryVoltage(), null);

        //Numbering system for drivetrain: 0 - front right, 1 - front left, 2 - back left, 3 - back right

        /* Front Left Metrics */
        builder.addDoubleProperty("Drivetrain Front Left Drive Current", () -> pdp.getCurrent(Constants.Swerve.Mod0.driveMotorID), null);
        builder.addDoubleProperty("Drivetrain Front Left Drive Temperature", () -> Swerve.getInstance().mSwerveMods[1].TalonDriveTemperature(), null);
        builder.addDoubleProperty("Drivetrain Front Left Angle Current", () -> pdp.getCurrent(Constants.Swerve.Mod0.angleMotorID), null);
        builder.addDoubleProperty("Drivetrain Front Left Angle Temperature", () -> Swerve.getInstance().mSwerveMods[1].TalonAngleTemperature(), null);
        builder.addDoubleProperty("Drivetrain Front Left Module Can Coder", () -> Swerve.getInstance().mSwerveMods[1].getCanCoder().getRotations(), null);
        /* Front Right Metrics */
        builder.addDoubleProperty("Drivetrain Front Right Drive Current", () -> pdp.getCurrent(Constants.Swerve.Mod1.driveMotorID), null);
        builder.addDoubleProperty("Drivetrain Front Right Drive Temperature", () -> Swerve.getInstance().mSwerveMods[0].TalonDriveTemperature(), null);
        builder.addDoubleProperty("Drivetrain Front Right Angle Current", () -> pdp.getCurrent(Constants.Swerve.Mod1.angleMotorID), null);
        builder.addDoubleProperty("Drivetrain Front Right Angle Temperature", () -> Swerve.getInstance().mSwerveMods[0].TalonAngleTemperature(), null);
        builder.addDoubleProperty("Drivetrain Front Right Module Can Coder", () -> Swerve.getInstance().mSwerveMods[0].getCanCoder().getRotations(), null);
        /* Back Left Metrics */
        builder.addDoubleProperty("Drivetrain Back Left Drive Current", () -> pdp.getCurrent(Constants.Swerve.Mod2.driveMotorID), null);
        builder.addDoubleProperty("Drivetrain Back Left Drive Temperature", () -> Swerve.getInstance().mSwerveMods[2].TalonDriveTemperature(), null);
        builder.addDoubleProperty("Drivetrain Back Left Angle Current", () -> pdp.getCurrent(Constants.Swerve.Mod2.angleMotorID), null);
        builder.addDoubleProperty("Drivetrain Back Left Angle Temperature", () -> Swerve.getInstance().mSwerveMods[2].TalonAngleTemperature(), null);
        builder.addDoubleProperty("Drivetrain Back Left Module Can Coder", () -> Swerve.getInstance().mSwerveMods[2].getCanCoder().getRotations(), null);
        /* Back Right Metrics */
        builder.addDoubleProperty("Drivetrain Back Right Drive Current", () -> pdp.getCurrent(Constants.Swerve.Mod3.driveMotorID), null);
        builder.addDoubleProperty("Drivetrain Back Right Drive Temperature", () -> Swerve.getInstance().mSwerveMods[3].TalonDriveTemperature(), null);
        builder.addDoubleProperty("Drivetrain Back Right Angle Current", () -> pdp.getCurrent(Constants.Swerve.Mod3.angleMotorID), null);
        builder.addDoubleProperty("Drivetrain Back Right Angle Temperature", () -> Swerve.getInstance().mSwerveMods[3].TalonAngleTemperature(), null);
        builder.addDoubleProperty("Drivetrain Back Right Module Can Coder", () -> Swerve.getInstance().mSwerveMods[3].getCanCoder().getRotations(), null);
        /* Shoulder Metrics */
        builder.addDoubleProperty("Arm Shoulder Motor Current", () -> pdp.getCurrent(Constants.Arm.shoulderMotorID), null);
        builder.addDoubleProperty("Arm Shoulder Motor Can Coder", () -> Arm.getInstance().getCanCoder("shoulder"), null);
        builder.addDoubleProperty("Arm Shoulder Motor Temperature", () -> Arm.getInstance().getTemperature("shoulder"), null);
        /* Elbow Metrics */
        builder.addDoubleProperty("Arm Elbow Motor Current", () -> pdp.getCurrent(Constants.Arm.elbowMotorID), null);
        builder.addDoubleProperty("Arm Elbow Motor Can Coder", () -> Arm.getInstance().getCanCoder("elbow"), null);
        builder.addDoubleProperty("Arm Elbow Motor Temperature", () -> Arm.getInstance().getTemperature("elbow"), null);

        super.initSendable(builder);
    }
}