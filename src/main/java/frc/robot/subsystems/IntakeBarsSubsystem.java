package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.IntakeBarsConstants;

public class IntakeBarsSubsystem extends SubsystemBase {
    private final TalonFX m_motor = new TalonFX(IntakeBarsConstants.kIntakeBarsMotorID);
    private final DutyCycleOut m_request = new DutyCycleOut(0);

    public IntakeBarsSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.StatorCurrentLimit = 40;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        m_motor.getConfigurator().apply(config);
    }

    /** Drive the bars at the given speed (-1.0 to 1.0). */
    public void setSpeed(double speed) {
        m_motor.setControl(m_request.withOutput(speed));
    }

    public void stop() {
        m_motor.setControl(m_request.withOutput(0));
    }

    // TODO: Tune fast/slow speeds
    public Command runFastCommand() {
        return this.startEnd(() -> setSpeed(0.9), () -> stop()).withName("Bars Fast");
    }

    public Command runSlowCommand() {
        return this.startEnd(() -> setSpeed(0.4), () -> stop()).withName("Bars Slow");
    }
}
