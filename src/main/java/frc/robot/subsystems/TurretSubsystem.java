package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLimitSwitch;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; 
import java.util.function.DoubleSupplier;

public class TurretSubsystem extends SubsystemBase {
    private SparkMax m_motor32;
    private RelativeEncoder m_encoder;
    private SparkLimitSwitch m_limitSwitch;
    private boolean m_isValid = false;

    public TurretSubsystem() {
        try {
            m_motor32 = new SparkMax(32, MotorType.kBrushless);
            m_encoder = m_motor32.getEncoder();
            m_limitSwitch = m_motor32.getReverseLimitSwitch();
            m_isValid = true;
            System.out.println("TURRET: Hardware initialized successfully.");
        } catch (Exception e) {
            m_isValid = false;
            System.out.println("TURRET ERROR: Could not find motor 32!");
        }
    }

    public void setRotation(double speed) {
        if (!m_isValid) return; // Stop the crash!
        
        if (m_limitSwitch.isPressed() && speed < 0) {
            m_motor32.set(0); 
        } else {
            m_motor32.set(speed);
        }
    }

    public Command findHomeCommand() {
        return this.run(() -> setRotation(-0.1))
            .until(() -> m_isValid && m_limitSwitch.isPressed())
            .finallyDo(() -> setRotation(0));
    }

    public Command rotateCommand(DoubleSupplier speedSupplier) {
        return this.run(() -> setRotation(speedSupplier.getAsDouble()))
                    .finallyDo(() -> setRotation(0));
    }

    @Override
    public void periodic() {
        if (m_isValid) {
            SmartDashboard.putNumber("Turret Pos", m_encoder.getPosition());
        }
    }
}