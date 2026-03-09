package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;

public class ShooterSubsystem extends SubsystemBase {
    // Define all three motors independently
    private final SparkMax m_motor20 = new SparkMax(20, MotorType.kBrushless);
    private final SparkMax m_motor21 = new SparkMax(21, MotorType.kBrushless);
    private final SparkMax m_motor22 = new SparkMax(22, MotorType.kBrushless);

    // Velocity PID controllers and encoders for motors 20 and 22
    private final SparkClosedLoopController m_controller20 = m_motor20.getClosedLoopController();
    private final SparkClosedLoopController m_controller22 = m_motor22.getClosedLoopController();
    private final RelativeEncoder m_encoder20 = m_motor20.getEncoder();
    private final RelativeEncoder m_encoder22 = m_motor22.getEncoder();

    // Aggressive velocity PID gains for shooter wheels (NEO ~5676 RPM free speed)
    private static final double kP      = 0.00016;
    private static final double kI      = 0.0;
    private static final double kD      = 0.0;
    private static final double kFF     = 0.00028; // 1 / 5676 RPM

    public ShooterSubsystem() {
        // --- Motor 21: open-loop only, standard config ---
        SparkMaxConfig config21 = new SparkMaxConfig();
        config21.idleMode(IdleMode.kCoast);
        config21.smartCurrentLimit(60);
        m_motor21.configure(config21, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // --- Motors 20 and 22: velocity PID config ---
        SparkMaxConfig pidConfig = new SparkMaxConfig();
        pidConfig.idleMode(IdleMode.kCoast);
        pidConfig.smartCurrentLimit(60);
        pidConfig.closedLoop
            .p(kP)
            .i(kI)
            .d(kD)
            .velocityFF(kFF)
            .outputRange(-1.0, 1.0);

        m_motor20.configure(pidConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_motor22.configure(pidConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * A flexible command to run all 3 motors at different speeds/directions.
     * Speed range is -1.0 to 1.0.
     */
    public Command runShooterCommand(double speed20, double speed21, double speed22) {
        System.out.println("runShooterCommand");        //JNL 02/11/2026 comment in message window
        return this.startEnd(
            () -> {
                m_motor20.set(speed20);
                m_motor21.set(speed21);
                m_motor22.set(speed22);
            },
            () -> {
                m_motor20.set(0);
                m_motor21.set(0);
                m_motor22.set(0);
            }
        ).withName("Run 3-Motor Shooter");
    }

    /**
     * Runs motors 20 and 22 at a target velocity using closed-loop PID,
     * and motor 21 at an open-loop percent output.
     *
     * @param rpm20   Target velocity for motor 20 in RPM
     * @param speed21 Open-loop percent output for motor 21 (-1.0 to 1.0)
     * @param rpm22   Target velocity for motor 22 in RPM
     */
    public Command runShooterRPMCommand(double rpm20, double speed21, double rpm22) {
        System.out.println("runShooterRPMCommand rpm20=" + rpm20 + " rpm22=" + rpm22);
        return this.startEnd(
            () -> {
                m_controller20.setReference(rpm20, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
                m_motor21.set(speed21);
                m_controller22.setReference(rpm22, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
            },
            () -> {
                m_motor20.set(0);
                m_motor21.set(0);
                m_motor22.set(0);
            }
        ).withName("Run Shooter RPM");
    }

    /** Returns the current velocity of motor 20 in RPM. */
    public double getVelocity20() {
        return m_encoder20.getVelocity();
    }

    /** Returns the current velocity of motor 22 in RPM. */
    public double getVelocity22() {
        return m_encoder22.getVelocity();
    }
}

