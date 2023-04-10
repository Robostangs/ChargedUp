package frc.robot.subsystems.Hand;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.LoggyThings.LoggyWPI_TalonFX;
import frc.robot.Constants;

public class Intake extends SubsystemBase{
    private static Intake mInstance;

    public static Intake getInstance() {
        if (mInstance == null) {
            mInstance = new Intake();
        }
        return mInstance;
    }
    
    private boolean holding;
    
    private LoggyWPI_TalonFX mIntakeMotor;

    private ArmFeedforward feedforward;

    public Intake() {
        holding = true;
        mIntakeMotor = new LoggyWPI_TalonFX(Constants.Hand.mIntakeMotor_ID);

        mIntakeMotor.setInverted(false);

        feedforward = new ArmFeedforward(0, 0, 0, 0);
    }

    public boolean getHolding() {
        return holding;
    }

    public void Suck() {
        mIntakeMotor.set(ControlMode.PercentOutput, Constants.Hand.kSuckSpeed);
    }

    public void Spit() {
        mIntakeMotor.set(ControlMode.PercentOutput, Constants.Hand.kSpitSpeed);
    }

    public void testBench() {
        // mIntakeMotor.set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward, feedforward.calculate(getWristAngle(), 0.5));
    }

    public void intakeFeedFwd() {
        mIntakeMotor.set(ControlMode.PercentOutput, 0.1);
    }

    public boolean isHolding() {
        return holding;
    }

    public void setHolding(boolean holding) {
        this.holding = holding;
    }
}
