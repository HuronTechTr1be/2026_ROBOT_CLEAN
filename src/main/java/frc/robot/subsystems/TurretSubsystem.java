package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    // 1. Define Variables
    private final SparkMax m_motor32;
    private final RelativeEncoder m_encoder;
    private final SparkClosedLoopController m_pidController;
    private final SparkLimitSwitch m_limitSwitch;

    // Math: 360 degrees / (4 gearbox * 6.5 turret reduction)
    private final double kDegreesPerRotation = 360.0 / (4.0 * 6.5);

    public TurretSubsystem() {
        m_motor32 = new SparkMax(32, MotorType.kBrushless);
        m_encoder = m_motor32.getEncoder();
        m_pidController = m_motor32.getClosedLoopController();
        m_limitSwitch = m_motor32.getReverseLimitSwitch();

        SparkMaxConfig config = new SparkMaxConfig();

        // 2. Configure Encoder
        config.encoder
            .positionConversionFactor(kDegreesPerRotation)
            .velocityConversionFactor(kDegreesPerRotation / 60.0);

        // 3. Configure PID Gains
        config.closedLoop
            .p(0.05) 
            .i(0.0)
            .d(0.0)
            .outputRange(-0.4, 0.4); 

        // 4. Force Hardware Limits OFF (so they don't stop the motor accidentally)
        config.limitSwitch
            .forwardLimitSwitchEnabled(false)
            .reverseLimitSwitchEnabled(false);
          
        // 5. Apply Config
        m_motor32.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Sets the target angle for the turret (Closed Loop PID).
     */
    public void setAngle(double degrees) {
        m_pidController.setReference(degrees, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    /**
     * Command to go to a specific angle.
     */
    public Command goToAngleCommand(double degrees) {
        return this.run(() -> setAngle(degrees));
    }

    /**
     * Homing Command: Spin slowly until limit switch is hit, then reset encoder to 0.
     */
    public Command findHomeCommand() {
        return this.run(() -> m_motor32.set(0.05))
            .until(() -> m_limitSwitch.isPressed())
            .finallyDo((interrupted) -> {
                m_motor32.set(0);
                if (!interrupted) {
                    m_encoder.setPosition(0); 
                }
            });
    }

    /**
     * Runs the turret motor at a specific speed (Manual / Open Loop).
     * @param speed -1.0 to 1.0
     */
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