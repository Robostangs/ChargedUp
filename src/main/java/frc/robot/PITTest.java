<<<<<<< Updated upstream
///*
package frc.robot;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Swerve;
import frc.robot.commands.AestheticsCMD.MusicCMD;
=======
package frc.robot;

import java.util.ArrayList;
import java.util.function.Consumer;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Hand;
import frc.robot.commands.AestheticsCMD.LightCMD;
import frc.robot.commands.AestheticsCMD.MusicCMD;
import frc.robot.commands.AestheticsCMD.WarningBeep;
import frc.robot.commands.Arm.PercentOutput;
>>>>>>> Stashed changes
import frc.robot.commands.Arm.SetArmPosition;
import frc.robot.commands.Hand.ToggleGrip;
import frc.robot.commands.Swerve.TeleopSwerve;
import frc.robot.subsystems.Arm;
<<<<<<< Updated upstream
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
        
        builder.addDoubleProperty("Arm Shoulder Motor Current", () -> pdp.getCurrent(Constants.Arm.shoulderMotorID), null);
        builder.addDoubleProperty("Arm Elbow Motor Current", () -> pdp.getCurrent(Constants.Arm.elbowMotorID), null);
        
        builder.addDoubleProperty("Arm Shoulder Motor Can Coder", () -> mArm.getCanCoder("shoulder"), null);
        builder.addDoubleProperty("Arm Elbow Motor Can Coder", () -> mArm.getCanCoder("elbow"), null);
        
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
=======
import frc.robot.subsystems.Aesthetics.Music;
import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.subsystems.Swerve;

public class PITTest extends SequentialCommandGroup {
    XboxController xDrive = new RobotContainer().mDriverController;
    double testSpeed = frc.robot.Constants.Swerve.testSpeed;

    SendableChooser<Command> chooser = new SendableChooser<Command>();

    public ArrayList<Command> testedCommands = new ArrayList<Command>();
    
    Command 
        translationTest, strafeTest, rotationTest,
        toggleGrip,
        setIntakePOS, setStowPOS, rawPower;

    Command[] commandList = {
        // /* Swerve testing */
        translationTest = new TeleopSwerve
            (() -> testSpeed, () -> 0, () -> 0, () -> false, () -> false),
        strafeTest = new TeleopSwerve
            (() -> 0, () -> testSpeed, () -> 0, () -> false, () -> false),
        rotationTest = new TeleopSwerve
            (() -> 0, () -> 0, () -> testSpeed, () -> false, () -> false),
        /* Arm Testing */
        toggleGrip = new ToggleGrip(),
        /* Hand Testing */
        setIntakePOS = new SetArmPosition(ArmPosition.kIntakePosition, false),
        setStowPOS = new SetArmPosition(ArmPosition.kStowPosition, true),
        rawPower = new PercentOutput(() -> xDrive.getRightY(), () -> xDrive.getLeftY())
    };

    int currCommand = 0;
    int offset = 0;
    static boolean ran = false;

    final Command init = new WaitCommand(1).andThen(new WarningBeep());
    BooleanConsumer x;


    public PITTest() {
        this.addRequirements(Swerve.getInstance(), Hand.getInstance(), Arm.getInstance());
        setName("PIT Test");
        Shuffleboard.getTab("Pit Test");
        Shuffleboard.selectTab("Pit Test");
        System.out.println("PITTEST");

        chooser.setDefaultOption("Default (Wait)", new WaitCommand(1));
        for (Command cmd : commandList) {
            chooser.addOption(cmd.getName(), cmd);
        }

        chooser.getSelected();

        this.addCommands(
            new InstantCommand(() -> {new LightCMD(0.73);}),
            init
            // new TeleopSwerve
            //     (() -> testSpeed, () -> 0, () -> 0, () -> false, () -> false),
            // new TeleopSwerve
            //     (() -> 0, () -> testSpeed, () -> 0, () -> false, () -> false),
            // new TeleopSwerve
            //     (() -> 0, () -> 0, () -> testSpeed, () -> false, () -> false),
            // /* Arm Testing */
            // new ToggleGrip(),
            // /* Hand Testing */
            // new SetArmPosition(ArmPosition.kIntakePosition, false),
            // new SetArmPosition(ArmPosition.kStowPosition, false),
            // new PercentOutput(() -> xDrive.getRightY(), () -> xDrive.getLeftY()),
            // new InstantCommand(() -> {ran = true;})
        );
    }

    public void exec2() {
        if (!chooser.getSelected().isScheduled()) {
            chooser.getSelected().schedule();
        }
    }

    public void exec() {
        if (xDrive.getAButtonPressed()) {
            this.addCommands(init, commandList[0]);
        } else if (xDrive.getBButtonPressed()) {
            x = new BooleanConsumer() {
                public void accept(boolean ran) {PITTest.ran = true;}                    
            };    
            this.finallyDo(x);
            this.end(false);
        } else if (xDrive.getRightBumperPressed()) {
            currCommand++;
            if (currCommand == commandList.length) {
                currCommand = commandList.length - 1;
            } else if (currCommand < 0) {
                currCommand = 0;
            } else {
                this.addCommands(init, commandList[currCommand]);
            }
        } else if (xDrive.getLeftBumperPressed()) {
            currCommand--;
            if (currCommand == commandList.length) {
                currCommand = commandList.length - 1;
            } else if (currCommand < 0) {
                currCommand = 0;
            } else {
                this.addCommands(init, commandList[currCommand]);
            }
        } else {
            if (commandList[currCommand].isFinished()) {
                this.andThen(commandList[currCommand]);
            }
        }
    }

    public String updateBoard() {
        if (currCommand == -1) {
            return "Press A to begin test";
        } else if (commandList[currCommand] == null) {
            return "null";
        } else {
            testedCommands.add(commandList[currCommand--]);
            SmartDashboard.putString("Tested Commands", testedCommands.toString());
            return commandList[currCommand].getName();
        }
>>>>>>> Stashed changes
    }

    public Command musicCommand() {
        final Command musicCMD = new MusicCMD();
<<<<<<< Updated upstream
        return musicCMD;
    }
}
=======
        new LightCMD(-0.81).schedule(); //Forrest Green Pallette
        return musicCMD;
    }

    public static boolean didRun() {
        return ran;
    }

    PowerDistribution pdp = Robot.pdh;

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
>>>>>>> Stashed changes
