package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Commands; // Added for simplified command composition
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TurretSubsystem extends SubsystemBase {
    private final SparkMax m_motor32;
    private final RelativeEncoder m_encoder;
    private final SparkClosedLoopController m_pidController;
    private final SparkLimitSwitch m_limitSwitch;

    private final double kDegreesPerRotation = 360.0 / (4.0 * 6.5);

    public TurretSubsystem() {
        m_motor32 = new SparkMax(32, MotorType.kBrushless);
        m_encoder = m_motor32.getEncoder();
        m_pidController = m_motor32.getClosedLoopController();
        
        // This is the REVERSE limit switch
        m_limitSwitch = m_motor32.getReverseLimitSwitch();

        SparkMaxConfig config = new SparkMaxConfig();

        config.encoder
            .positionConversionFactor(kDegreesPerRotation)
            .velocityConversionFactor(kDegreesPerRotation / 60.0);

        config.closedLoop
            .p(0.18)
            .i(0.0)
            .d(0.0)
            .outputRange(-0.4, 0.4); 

        config.limitSwitch
            .forwardLimitSwitchEnabled(false)
            .reverseLimitSwitchEnabled(true); // Hardware will stop motor at negative speeds

        config.softLimit
            .forwardSoftLimit(170)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(-170)
            .reverseSoftLimitEnabled(true);

        m_motor32.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setAngle(double degrees) {
        m_pidController.setReference(degrees, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public Command goToAngleCommand(double degrees) {
        return this.run(() -> setAngle(degrees));
    }

    /**
     * Improved Homing: 
     * 1. Checks if already on switch. If so, reset and finish.
     * 2. If not, drive reverse (-0.05) until switch is hit.
     */
    public Command findHomeCommand() {
        return Commands.either(
            // If already pressed:
            this.runOnce(() -> {
                m_encoder.setPosition(0);
                System.out.println("Turret already home. Resetting encoder.");
            }),
            // If NOT pressed, run the movement sequence:
            this.run(() -> m_motor32.set(-0.05)) // Negative speed for reverse switch
                .until(() -> m_limitSwitch.isPressed())
                .finallyDo((interrupted) -> {
                    m_motor32.set(0);
                    if (!interrupted) {
                        m_encoder.setPosition(0);
                        System.out.println("Turret homing complete.");
                    }
                }),
            m_limitSwitch::isPressed
        );
    }

    public void runManual(double speed) {
        m_motor32.set(speed); 
    }

    public void stop() {
        m_motor32.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Turret/Angle", m_encoder.getPosition());
        SmartDashboard.putBoolean("Turret/Home", m_limitSwitch.isPressed());
    }
}