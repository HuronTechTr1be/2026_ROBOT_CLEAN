package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeSubsystem extends SubsystemBase {

    private final TalonFX m_frontRoller = new TalonFX(13);
    private final TalonFX m_backRoller  = new TalonFX(14);
    private final TalonFX m_Roller      = new TalonFX(15);

    private final DutyCycleOut m_frontRequest  = new DutyCycleOut(0);
    private final DutyCycleOut m_backRequest   = new DutyCycleOut(0);
    private final DutyCycleOut m_rollerRequest = new DutyCycleOut(0);

    public IntakeSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.StatorCurrentLimit = 60;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        m_frontRoller.getConfigurator().apply(config);
        m_backRoller.getConfigurator().apply(config);
        m_Roller.getConfigurator().apply(config);
    }

    /**
     * Run both intake rollers at independent speeds.
     * @param frontSpeed Speed for the front roller (-1.0 to 1.0)
     * @param backSpeed  Speed for the back roller (-1.0 to 1.0)
     */
    public Command runIntakeCommand(double frontSpeed, double backSpeed) {
        return this.startEnd(
            () -> {
                m_frontRoller.setControl(m_frontRequest.withOutput(frontSpeed));
                m_backRoller.setControl(m_backRequest.withOutput(backSpeed));
                m_Roller.setControl(m_rollerRequest.withOutput(frontSpeed));
            },
            () -> {
                m_frontRoller.setControl(m_frontRequest.withOutput(0));
                m_backRoller.setControl(m_backRequest.withOutput(0));
                m_Roller.setControl(m_rollerRequest.withOutput(0));
            }
        ).withName("Run Intake Dual");
    }
}
