package frc.robot.Test;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.Arm.ArmManager;
import frc.robot.commands.Drivetrain.TeleopSwerve;
import frc.robot.commands.Hand.HandManager;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Hand.HandState;

public class PITTest extends SequentialCommandGroup {
    PowerDistribution pdp = Robot.mPowerDistributionPanel;
    boolean ran = false;

    public PITTest() {
        this.addRequirements();
        setName("PIT Test");
        SmartDashboard.putString("PIT Test", "Test Starting");
    
        this.addCommands(
            new TeleopSwerve(() -> 0.2, () -> 0, () -> 0, () -> false),
            new TeleopSwerve(() -> 0, () -> 0.2, () -> 0, () -> false),
            new TeleopSwerve(() -> 0, () -> 0, () -> 0.2, () -> false),

            new HandManager(HandState.OPEN),
            new HandManager(HandState.CLOSED),

            new ArmManager(ArmState.kIntakePosition),
            new ArmManager(ArmState.kLoadingZonePosition),
            new ArmManager(ArmState.kStowedPosition),
            
            new InstantCommand(() -> {ran = true;})
        );
        
    }

    @Override
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

        super.initSendable(builder);
    } 

    public boolean didRun() {
        return ran;
    }
}
