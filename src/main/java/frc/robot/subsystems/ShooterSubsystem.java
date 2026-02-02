package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class ShooterSubsystem extends SubsystemBase {
    // Define all three motors independently
    private final SparkMax m_motor20 = new SparkMax(20, MotorType.kBrushless);
    private final SparkMax m_motor21 = new SparkMax(21, MotorType.kBrushless);
    private final SparkMax m_motor22 = new SparkMax(22, MotorType.kBrushless);

    public ShooterSubsystem() {
        // Create a standard configuration for shooter motors
        SparkMaxConfig shooterConfig = new SparkMaxConfig();
        shooterConfig.idleMode(IdleMode.kCoast); // Coast is best for shooters
        shooterConfig.smartCurrentLimit(60);

        // Apply this config to ALL motors so they behave the same way safely
        m_motor20.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_motor21.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_motor22.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * A flexible command to run all 3 motors at different speeds/directions.
     * Speed range is -1.0 to 1.0.
     */
    public Command runShooterCommand(double speed20, double speed21, double speed22) {
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
}