/*package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ShooterSubsystem extends SubsystemBase {
    // CAN IDs preserved: 20, 21, 22
    private final TalonFX m_motor20 = new TalonFX(20);
    private final TalonFX m_motor21 = new TalonFX(21);
    private final TalonFX m_motor22 = new TalonFX(22);

    private final DutyCycleOut    m_dutyCycle20 = new DutyCycleOut(0);
    private final DutyCycleOut    m_dutyCycle21 = new DutyCycleOut(0);
    private final DutyCycleOut    m_dutyCycle22 = new DutyCycleOut(0);
    private final VelocityDutyCycle m_velocity20 = new VelocityDutyCycle(0);
    private final VelocityDutyCycle m_velocity22 = new VelocityDutyCycle(0);

    // NOTE: These gains are approximate conversions from SparkMax NEO values.
    // They MUST be re-tuned for Kraken X60. Phoenix 6 velocity units are RPS (rotations per second).
    // Original: kP = 0.00016/RPM -> converted ~0.0096/RPS; kFF = 0.00028/RPM -> kV ~0.0168/RPS
    private static final double kP = 0.0096;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kV = 0.0168; // feedforward: duty cycle per RPS

    public ShooterSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.StatorCurrentLimit = 60;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        // Motor 21: open-loop only — no PID slot needed
        m_motor21.getConfigurator().apply(config);

        // Motors 20 and 22: velocity PID via Slot 0
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        config.Slot0.kV = kV;
        m_motor20.getConfigurator().apply(config);
        m_motor22.getConfigurator().apply(config);
    }

    /**
     * A flexible command to run all 3 motors at different speeds/directions.
     * Speed range is -1.0 to 1.0.
     
    /public Command runShooterCommand(double speed20, double speed21, double speed22) {
        System.out.println("runShooterCommand"); // JNL 02/11/2026 comment in message window
        return this.startEnd(
            () -> {
                m_motor20.setControl(m_dutyCycle20.withOutput(speed20));
                m_motor21.setControl(m_dutyCycle21.withOutput(speed21));
                m_motor22.setControl(m_dutyCycle22.withOutput(speed22));
            },
            () -> {
                m_motor20.setControl(m_dutyCycle20.withOutput(0));
                m_motor21.setControl(m_dutyCycle21.withOutput(0));
                m_motor22.setControl(m_dutyCycle22.withOutput(0));
            }
        ).withName("Run 3-Motor Shooter");
    }

    /**
     * Runs motors 20 and 22 at a target velocity using closed-loop VelocityDutyCycle,
     * and motor 21 at an open-loop percent output.
     *
     * @param rpm20   Target velocity for motor 20 in RPM
     * @param speed21 Open-loop percent output for motor 21 (-1.0 to 1.0)
     * @param rpm22   Target velocity for motor 22 in RPM
     
    public Command runShooterRPMCommand(double rpm20, double speed21, double rpm22) {
        System.out.println("runShooterRPMCommand rpm20=" + rpm20 + " rpm22=" + rpm22);
        return this.startEnd(
            () -> {
                // Phoenix 6 velocity is in RPS; convert from RPM
                m_motor20.setControl(m_velocity20.withVelocity(rpm20 / 60.0));
                m_motor21.setControl(m_dutyCycle21.withOutput(speed21));
                m_motor22.setControl(m_velocity22.withVelocity(rpm22 / 60.0));
            },
            () -> {
                m_motor20.setControl(m_dutyCycle20.withOutput(0));
                m_motor21.setControl(m_dutyCycle21.withOutput(0));
                m_motor22.setControl(m_dutyCycle22.withOutput(0));
            }
        ).withName("Run Shooter RPM");
    }

     Returns the current velocity of motor 20 in RPM. */
    //public double getVelocity20() {
        // Phoenix 6 getVelocity() returns RPS; convert to RPM for backwards compatibility
        //return m_motor20.getVelocity().getValueAsDouble() * 60.0;
    //}

     //Returns the current velocity of motor 22 in RPM. */
    //public double getVelocity22() {
        //return m_motor22.getVelocity().getValueAsDouble() * 60.0;
  //  }
//}
  