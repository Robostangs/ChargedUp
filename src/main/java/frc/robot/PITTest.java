package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.Arm.PercentOutput;
import frc.robot.commands.Arm.ProfiledChangeSetPoint;
import frc.robot.commands.Hand.SetGrip;
import frc.robot.commands.Hand.ToggleGrip;
import frc.robot.commands.Hand.ToggleHolding;
import frc.robot.commands.Lights.LightCMD;
import frc.robot.commands.Swerve.TeleopSwerve;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Hand;
import frc.robot.subsystems.Lighting;
import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.subsystems.Swerve;

public class PITTest {
    static XboxController xDrive = RobotContainer.mDriverController;
    static double testSpeed = frc.robot.Constants.Swerve.testSpeed;
    static PowerDistribution pdp = Robot.mPowerDistribution;
    static Double maxSpeed = 1.0;
    static Double minSpeed = -1.0;
    static WaitCommand buffer = new WaitCommand(4);
    
    public static String[] cmdList = {
        "Translation Test", "Strafe Test", "Rotation Test", "Drivetrain Control",
        "Toggle Claw",
        "General Intake Position", "Stow Position", "Fine Adjust"
    };
    
    public static Command[] commandList = {
        /* Swerve testing */
        // new InstantCommand(() -> {RobotContainer.s_Swerve.setDefaultCommand(new TeleopSwerve(() -> testSpeed, () -> 0, () -> 0, () -> false, () -> false));}),


        new TeleopSwerve(() -> testSpeed, () -> 0, () -> 0, () -> false, () -> false),
        new TeleopSwerve(() -> 0, () -> testSpeed, () -> 0, () -> false, () -> false),
        new TeleopSwerve(() -> 0, () -> 0, () -> testSpeed, () -> false, () -> false),
        new TeleopSwerve(
            () -> -xDrive.getLeftY(), 
            () -> -xDrive.getLeftX(), 
            () -> -xDrive.getRightX(), 
            () ->  xDrive.getAButton(),
            () ->  xDrive.getLeftBumper()
        ),
        /* Arm Testing */
        // new InstantCommand(() -> {RobotContainer.s_Hand.setDefaultCommand(new ToggleGrip());}),

        // new SequentialCommandGroupToggleGrip().alongWith(new WaitUntilCommand(4)).withName(null),
        new ToggleHolding().andThen(new WaitCommand(2)),
        // .andThen(new WaitCommand(4)),
        /* Hand Testing */
        // new InstantCommand(() -> {if (Arm.getInstance().getArmPosition().equals(ArmPosition.kIntakePositionGeneral)) {new InstantCommand(() -> System.out.println("Intake Position General"));} else {new SetArmPosition(ArmPosition.kIntakePositionGeneral);}}),
        new ConditionalCommand(
            new InstantCommand(), new InstantCommand(), () -> true),
            // new InstantCommand(() -> System.out.println("Intake Position General")),
            // new SetArmPosition(ArmPosition.kIntakePositionGeneral),
            // () -> Arm.getInstance().getArmPosition().equals(ArmPosition.kIntakePositionGeneral)),
        new ConditionalCommand(
            new InstantCommand(), new InstantCommand(), () -> true),
            // new InstantCommand(() -> System.out.println("Stow Position")),
            // new SetArmPosition(ArmPosition.kStowPosition),
            // () -> Arm.getInstance().getArmPosition().equals(ArmPosition.kStowPosition)),
        new ConditionalCommand(
            new InstantCommand(), new InstantCommand(), () -> true),
            // new InstantCommand(() -> System.out.println("Stow Position")),
            // ProfiledChangeSetPoint.createWithTimeout(() -> Constants.Arm.SetPoint.stowPosition),
            // () -> (Arm.getInstance().getArmPosition().equals(ArmPosition.kStowPosition))),
        // .andThen(new WaitCommand(4)),
        // new SetArmPosition(ArmPosition.kStowPosition),
        // .andThen(new WaitCommand(4)),
        new PercentOutput(
            () -> Utils.customDeadzone(-xDrive.getLeftY()),
            () -> Utils.customDeadzone(-xDrive.getRightY()))
    };      //TODO: VERY IMPORTANT PROBLEM, TOGGLEGRIP THRU PERCENT OUTPUT ARE NOT BEING DONE, EX: NOT SHOWING UP AS CURRENT COMMAND

    private static DoubleSupplier BV;
    private static DoubleSupplier FL1, FL2, FL3, FL4, FL5;
    private static DoubleSupplier FR1, FR2, FR3, FR4, FR5;
    private static DoubleSupplier BL1, BL2, BL3, BL4, BL5; 
    private static DoubleSupplier BR1, BR2, BR3, BR4, BR5;
    private static DoubleSupplier Shoulder1, Shoulder2, Shoulder3;
    private static DoubleSupplier Elbow1, Elbow2, Elbow3;

    public PITTest() {
        buffer.addRequirements(Swerve.getInstance(), Arm.getInstance(), Hand.getInstance(), Lighting.getInstance());
        new InstantCommand(() -> Swerve.getInstance());
        new InstantCommand(() -> Arm.getInstance());
        new InstantCommand(() -> Hand.getInstance());
        new InstantCommand(() -> Lighting.getInstance());
        
        Swerve.getInstance().removeDefaultCommand();
        Arm.getInstance().removeDefaultCommand();
        Hand.getInstance().removeDefaultCommand();
        Lighting.getInstance().removeDefaultCommand();
        
        new InstantCommand(() -> new LightCMD(Lighting.kKillLights)).schedule();
        Swerve.getInstance().setDefaultCommand(new TeleopSwerve(() -> 0, () -> 0, () -> 0, () -> false, () -> false).withName("Kill Drivetrain"));
        Arm.getInstance().setDefaultCommand(new PercentOutput(() -> 0, () -> 0).withName("Kill Arm"));
        // Hand.getInstance().setDefaultCommand(new SetGrip().withName("Close Claw"));
        System.out.println("PITTest.PITTest()");

        // LiveWindow.setEnabled(false);
    }

    public void speedPlus() {
        if (Constants.Swerve.testSpeed + 0.1 < maxSpeed) {
            Constants.Swerve.testSpeed = Constants.Swerve.testSpeed + 0.1;
        }
        SmartDashboard.putNumber("Test Speed", Constants.Swerve.testSpeed);
    }

    public void speedMinus() {
        if (Constants.Swerve.testSpeed - 0.1 > minSpeed) {
            Constants.Swerve.testSpeed = Constants.Swerve.testSpeed - 0.1;
        }
        SmartDashboard.putNumber("Test Speed", Constants.Swerve.testSpeed);
    }

//     public static void init() {
//         BV = () -> RobotController.getBatteryVoltage();
//         /* Front Left Metrics Mod[1] */
//         FL1 = () -> pdp.getCurrent(Constants.Swerve.Mod1.driveMotorID);
//         FL2 = () -> Swerve.getInstance().mSwerveMods[1].TalonDriveTemperature();
//         FL3 = () -> pdp.getCurrent(Constants.Swerve.Mod1.angleMotorID);
//         FL4 = () -> Swerve.getInstance().mSwerveMods[1].TalonAngleTemperature();
//         FL5 = () -> Swerve.getInstance().mSwerveMods[1].getCanCoder().getRotations();
//         /* Front Right Metrics Mod[0] */
//         FR1 = () -> pdp.getCurrent(Constants.Swerve.Mod0.driveMotorID);
//         FR2 = () -> Swerve.getInstance().mSwerveMods[0].TalonDriveTemperature();
//         FR3 = () -> pdp.getCurrent(Constants.Swerve.Mod0.angleMotorID);
//         FR4 = () -> Swerve.getInstance().mSwerveMods[0].TalonAngleTemperature();
//         FR5 = () -> Swerve.getInstance().mSwerveMods[0].getCanCoder().getRotations();
//         /* Back Left Metrics Mod[2] */
//         BL1 = () -> pdp.getCurrent(Constants.Swerve.Mod2.driveMotorID);
//         BL2 = () -> Swerve.getInstance().mSwerveMods[2].TalonDriveTemperature();
//         BL3 = () -> pdp.getCurrent(Constants.Swerve.Mod2.angleMotorID);
//         BL4 = () -> Swerve.getInstance().mSwerveMods[2].TalonAngleTemperature();
//         BL5 = () -> Swerve.getInstance().mSwerveMods[2].getCanCoder().getRotations();
//         /* Back Right Metrics Mod[3] */
//         BR1 = () -> pdp.getCurrent(Constants.Swerve.Mod3.driveMotorID);
//         BR2 = () -> Swerve.getInstance().mSwerveMods[3].TalonDriveTemperature();
//         BR3 = () -> pdp.getCurrent(Constants.Swerve.Mod3.angleMotorID);
//         BR4 = () -> Swerve.getInstance().mSwerveMods[3].TalonAngleTemperature();
//         BR5 = () -> Swerve.getInstance().mSwerveMods[3].getCanCoder().getRotations();
//         /* Shoulder Metrics */
//         Shoulder1 = () -> pdp.getCurrent(Constants.Arm.shoulderMotorID);
//         Shoulder2 = () -> Arm.getInstance().getAbsolutePositionShoulder();
//         Shoulder3 = () -> Arm.getInstance().getTemperature("shoulder");
//         /* Elbow Metrics */
//         Elbow1 = () -> pdp.getCurrent(Constants.Arm.elbowMotorID);
//         Elbow2 = () -> Arm.getInstance().getAbsolutePositionElbow();
//         Elbow3 = () -> Arm.getInstance().getTemperature("elbow");
//     }

    public static void PDH() {
        SmartDashboard.putNumber("Battery Voltage", BV.getAsDouble());

//         /* Front Left Metrics Mod[1] */
//         SmartDashboard.putNumber("Drivetrain Front Left Drive Current", FL1.getAsDouble());
//         SmartDashboard.putNumber("Drivetrain Front Left Drive Temperature", FL2.getAsDouble());
//         SmartDashboard.putNumber("Drivetrain Front Left Angle Current", FL3.getAsDouble());
//         SmartDashboard.putNumber("Drivetrain Front Left Angle Temperature", FL4.getAsDouble());
//         SmartDashboard.putNumber("Drivetrain Front Left Module Can Coder", FL5.getAsDouble());
//         /* Front Right Metrics Mod[0] */
//         SmartDashboard.putNumber("Drivetrain Front Right Drive Current", FR1.getAsDouble());
//         SmartDashboard.putNumber("Drivetrain Front Right Drive Temperature", FR2.getAsDouble());
//         SmartDashboard.putNumber("Drivetrain Front Right Angle Current", FR3.getAsDouble());
//         SmartDashboard.putNumber("Drivetrain Front Right Angle Temperature", FR4.getAsDouble());
//         SmartDashboard.putNumber("Drivetrain Front Right Module Can Coder", FR5.getAsDouble());
//         /* Back Left Metrics Mod[2] */
//         SmartDashboard.putNumber("Drivetrain Back Left Drive Current", BL1.getAsDouble());
//         SmartDashboard.putNumber("Drivetrain Back Left Drive Temperature", BL2.getAsDouble());
//         SmartDashboard.putNumber("Drivetrain Back Left Angle Current", BL3.getAsDouble());
//         SmartDashboard.putNumber("Drivetrain Back Left Angle Temperature", BL4.getAsDouble());
//         SmartDashboard.putNumber("Drivetrain Back Left Module Can Coder", BL5.getAsDouble());
//         /* Back Right Metrics Mod[3] */
//         SmartDashboard.putNumber("Drivetrain Back Right Drive Current", BR1.getAsDouble());
//         SmartDashboard.putNumber("Drivetrain Back Right Drive Temperature", BR2.getAsDouble());
//         SmartDashboard.putNumber("Drivetrain Back Right Angle Current", BR3.getAsDouble());
//         SmartDashboard.putNumber("Drivetrain Back Right Angle Temperature", BR4.getAsDouble());
//         SmartDashboard.putNumber("Drivetrain Back Right Module Can Coder", BR5.getAsDouble());
//         /* Shoulder Metrics */
//         SmartDashboard.putNumber("Shoulder Motor Current", Shoulder1.getAsDouble());
//         SmartDashboard.putNumber("Shoulder Motor Can Coder", Shoulder2.getAsDouble());
//         SmartDashboard.putNumber("Shoulder Motor Temperature", Shoulder3.getAsDouble());
//         /* Elbow Metrics */
//         SmartDashboard.putNumber("Elbow Motor Current", Elbow1.getAsDouble());
//         SmartDashboard.putNumber("Elbow Motor Can Coder", Elbow2.getAsDouble());
//         SmartDashboard.putNumber("Elbow Motor Temperature", Elbow3.getAsDouble());
//     }



//     // @Override
//     // public void initSendable(SendableBuilder builder) {
//     //     builder.addDoubleProperty("Battery Voltage", () -> RobotController.getBatteryVoltage(), null);

//     //     //Numbering system for drivetrain: 0 - front right, 1 - front left, 2 - back left, 3 - back right

//     //     /* Front Left Metrics */
//     //     builder.addDoubleProperty("Drivetrain Front Left Drive Current", () -> pdp.getCurrent(Constants.Swerve.Mod0.driveMotorID), null);
//     //     builder.addDoubleProperty("Drivetrain Front Left Drive Temperature", () -> Swerve.getInstance().mSwerveMods[1].TalonDriveTemperature(), null);
//     //     builder.addDoubleProperty("Drivetrain Front Left Angle Current", () -> pdp.getCurrent(Constants.Swerve.Mod0.angleMotorID), null);
//     //     builder.addDoubleProperty("Drivetrain Front Left Angle Temperature", () -> Swerve.getInstance().mSwerveMods[1].TalonAngleTemperature(), null);
//     //     builder.addDoubleProperty("Drivetrain Front Left Module Can Coder", () -> Swerve.getInstance().mSwerveMods[1].getCanCoder().getRotations(), null);
//     //     /* Front Right Metrics */
//     //     builder.addDoubleProperty("Drivetrain Front Right Drive Current", () -> pdp.getCurrent(Constants.Swerve.Mod1.driveMotorID), null);
//     //     builder.addDoubleProperty("Drivetrain Front Right Drive Temperature", () -> Swerve.getInstance().mSwerveMods[0].TalonDriveTemperature(), null);
//     //     builder.addDoubleProperty("Drivetrain Front Right Angle Current", () -> pdp.getCurrent(Constants.Swerve.Mod1.angleMotorID), null);
//     //     builder.addDoubleProperty("Drivetrain Front Right Angle Temperature", () -> Swerve.getInstance().mSwerveMods[0].TalonAngleTemperature(), null);
//     //     builder.addDoubleProperty("Drivetrain Front Right Module Can Coder", () -> Swerve.getInstance().mSwerveMods[0].getCanCoder().getRotations(), null);
//     //     /* Back Left Metrics */
//     //     builder.addDoubleProperty("Drivetrain Back Left Drive Current", () -> pdp.getCurrent(Constants.Swerve.Mod2.driveMotorID), null);
//     //     builder.addDoubleProperty("Drivetrain Back Left Drive Temperature", () -> Swerve.getInstance().mSwerveMods[2].TalonDriveTemperature(), null);
//     //     builder.addDoubleProperty("Drivetrain Back Left Angle Current", () -> pdp.getCurrent(Constants.Swerve.Mod2.angleMotorID), null);
//     //     builder.addDoubleProperty("Drivetrain Back Left Angle Temperature", () -> Swerve.getInstance().mSwerveMods[2].TalonAngleTemperature(), null);
//     //     builder.addDoubleProperty("Drivetrain Back Left Module Can Coder", () -> Swerve.getInstance().mSwerveMods[2].getCanCoder().getRotations(), null);
//     //     /* Back Right Metrics */
//     //     builder.addDoubleProperty("Drivetrain Back Right Drive Current", () -> pdp.getCurrent(Constants.Swerve.Mod3.driveMotorID), null);
//     //     builder.addDoubleProperty("Drivetrain Back Right Drive Temperature", () -> Swerve.getInstance().mSwerveMods[3].TalonDriveTemperature(), null);
//     //     builder.addDoubleProperty("Drivetrain Back Right Angle Current", () -> pdp.getCurrent(Constants.Swerve.Mod3.angleMotorID), null);
//     //     builder.addDoubleProperty("Drivetrain Back Right Angle Temperature", () -> Swerve.getInstance().mSwerveMods[3].TalonAngleTemperature(), null);
//     //     builder.addDoubleProperty("Drivetrain Back Right Module Can Coder", () -> Swerve.getInstance().mSwerveMods[3].getCanCoder().getRotations(), null);
//     //     /* Shoulder Metrics */
//     //     builder.addDoubleProperty("Arm Shoulder Motor Current", () -> pdp.getCurrent(Constants.Arm.shoulderMotorID), null);
//     //     builder.addDoubleProperty("Arm Shoulder Motor Can Coder", () -> Arm.getInstance().getCanCoder("shoulder"), null);
//     //     builder.addDoubleProperty("Arm Shoulder Motor Temperature", () -> Arm.getInstance().getTemperature("shoulder"), null);
//     //     /* Elbow Metrics */
//     //     builder.addDoubleProperty("Arm Elbow Motor Current", () -> pdp.getCurrent(Constants.Arm.elbowMotorID), null);
//     //     builder.addDoubleProperty("Arm Elbow Motor Can Coder", () -> Arm.getInstance().getCanCoder("elbow"), null);
//     //     builder.addDoubleProperty("Arm Elbow Motor Temperature", () -> Arm.getInstance().getTemperature("elbow"), null);

//     //     super.initSendable(builder);
    // }
}
}
