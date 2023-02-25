///*
package frc.robot;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Swerve;
import frc.robot.commands.Arm.SetArmPosition;
import frc.robot.commands.Hand.ToggleGrip;
import frc.robot.commands.Swerve.TeleopSwerve;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmPosition;


public class PITTest {
    private frc.robot.subsystems.Swerve mSwerve = new frc.robot.subsystems.Swerve();
    private Arm mArm = new Arm();

    private PowerDistribution pdp = new PowerDistribution();
    private boolean ran = false;

    /* Swerve testing */
    final Command translationTest = new TeleopSwerve(() -> Swerve.testSpeed, () -> 0, () -> 0, () -> false);
    final Command strafeTest = new TeleopSwerve(() -> 0, () -> Swerve.testSpeed, () -> 0, () -> false);
    final Command rotationTest = new TeleopSwerve(() -> 0, () -> 0, () -> Swerve.testSpeed, () -> false);

    /* Arm Testing */
    final Command toggleGrip = new ToggleGrip();

    /* Hand Testing */
    final Command setStowPOS = new SetArmPosition(ArmPosition.kStowPosition, false);
    final Command setIntakePOS = new SetArmPosition(ArmPosition.kIntakePosition, false);
    final Command setLoadingZonePOS = new SetArmPosition(ArmPosition.kLoadingZonePosition, false);
    final Command setLowPOS = new SetArmPosition(ArmPosition.kLowPosition, false);
    final Command setMediumPOS = new SetArmPosition(ArmPosition.kMediumPosition, false);
    final Command setHighPOS = new SetArmPosition(ArmPosition.kHighPosition, false);

    public ArrayList<Command> testedCommands = new ArrayList<Command>();
    //public String[] testedCommandsString = {};

    private Command[] commandlist = {
        /* Swerve testing */
        translationTest,
        strafeTest,
        rotationTest,

        /* Arm Testing */
        setStowPOS, 
        setIntakePOS,
 
        /* Hand Testing */
        toggleGrip,
        toggleGrip
    };

    private BooleanSupplier input;
    private Command currCommand;

    private int commandNum = 0;

    public PITTest() {
        Shuffleboard.getTab("Pit Test");
        SmartDashboard.putString("PIT Test Status", "Initizalizing");
        input = (() -> SmartDashboard.getBoolean("Next Command", false));
        scheduleCommand(commandNum).schedule();
    }

    public void execute() {
        if (input.getAsBoolean()) {
            testedCommands.add(currCommand);
            //testedCommandsString = (String[]) testedCommands.toArray();
            SmartDashboard.putStringArray("Tested Commands", (String[]) testedCommands.toArray());

            scheduleCommand(commandNum++).schedule();
            SmartDashboard.putBoolean("Next Command", false);
        }
        if (commandlist[commandNum].isFinished()) {
            scheduleCommand(commandNum).schedule();
        }
    }

    //@Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Battery Voltage", () -> RobotController.getBatteryVoltage(), null);
        
        builder.addDoubleProperty("Drivetrain Front Left Drive Current", () -> pdp.getCurrent(Constants.Swerve.Mod0.driveMotorID), null);
        builder.addDoubleProperty("Drivetrain Front Left Angle Current", () -> pdp.getCurrent(Constants.Swerve.Mod0.angleMotorID), null);
        
        builder.addDoubleProperty("Drivetrain Front Right Drive Current", () -> pdp.getCurrent(Constants.Swerve.Mod1.driveMotorID), null);
        builder.addDoubleProperty("Drivetrain Front Right Angle Current", () -> pdp.getCurrent(Constants.Swerve.Mod1.angleMotorID), null);
        
        builder.addDoubleProperty("Drivetrain Back Left Drive Current", () -> pdp.getCurrent(Constants.Swerve.Mod2.driveMotorID), null);
        builder.addDoubleProperty("Drivetrain Back Left Angle Current", () -> pdp.getCurrent(Constants.Swerve.Mod2.angleMotorID), null);
        
        builder.addDoubleProperty("Drivetrain Back Right Drive Current", () -> pdp.getCurrent(Constants.Swerve.Mod3.driveMotorID), null);
        builder.addDoubleProperty("Drivetrain Back Right Angle Current", () -> pdp.getCurrent(Constants.Swerve.Mod3.angleMotorID), null);
        
        builder.addDoubleProperty("Arm Shoulder Motor", () -> pdp.getCurrent(Constants.Arm.shoulderMotorID), null);
        builder.addDoubleProperty("Arm Elbow Motor", () -> pdp.getCurrent(Constants.Arm.elbowMotorID), null);
        
        builder.addDoubleProperty("Arm Shoulder Motor Encoder", () -> mArm.getCanCoder("shoulder"), null);
        builder.addDoubleProperty("Arm Elbow Motor Encoder", () -> mArm.getCanCoder("elbow"), null);
        
        builder.addDoubleProperty("Arm Shoulder Motor Temperature", () -> mArm.getTemperature("shoulder"), null);
        builder.addDoubleProperty("Arm Elbow Motor Temperature", () -> mArm.getTemperature("elbow"), null);
        
        //Numbering system for drivetrain: 0 - front right, 1 - front left, 2 - back left, 3 - back right

        builder.addDoubleProperty("Drivetrain Front Left Drive Temperature", () -> mSwerve.mSwerveMods[1].TalonDriveTemperature(), null);
        builder.addDoubleProperty("Drivetrain Front Left Angle Temperature", () -> mSwerve.mSwerveMods[1].TalonAngleTemperature(), null);
        builder.addDoubleProperty("Drivetrain Front Left Drive Can Coder", () -> mSwerve.mSwerveMods[1].getPosition().distanceMeters, null);
        
        builder.addDoubleProperty("Drivetrain Front Right Drive Temperature", () -> mSwerve.mSwerveMods[0].TalonDriveTemperature(), null);
        builder.addDoubleProperty("Drivetrain Front Right Angle Temperature", () -> mSwerve.mSwerveMods[0].TalonAngleTemperature(), null);
        builder.addDoubleProperty("Drivetrain Front Right Drive Can Coder", () -> mSwerve.mSwerveMods[0].getPosition().distanceMeters, null);
        
        builder.addDoubleProperty("Drivetrain Back Left Drive Temperature", () -> mSwerve.mSwerveMods[2].TalonDriveTemperature(), null);
        builder.addDoubleProperty("Drivetrain Back Left Angle Temperature", () -> mSwerve.mSwerveMods[2].TalonAngleTemperature(), null);
        builder.addDoubleProperty("Drivetrain Back Left Drive Can Coder", () -> mSwerve.mSwerveMods[2].getPosition().distanceMeters, null);
        
        builder.addDoubleProperty("Drivetrain Back Right Drive Temperature", () -> mSwerve.mSwerveMods[3].TalonDriveTemperature(), null);
        builder.addDoubleProperty("Drivetrain Back Right Angle Temperature", () -> mSwerve.mSwerveMods[3].TalonAngleTemperature(), null);
        builder.addDoubleProperty("Drivetrain Back Right Drive Can Coder", () -> mSwerve.mSwerveMods[3].getPosition().distanceMeters, null);
    }

    public boolean didRun() {
        return ran;
    }

    private Command scheduleCommand(int commandNum) {
        this.commandNum = commandNum;
        Command x = commandlist[commandNum];
        currCommand = x;
        SmartDashboard.putString("PIT Test Status", currCommand.getName());
        return x;
    }
}
