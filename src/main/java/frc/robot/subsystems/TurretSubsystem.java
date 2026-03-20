/*package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

public class TurretSubsystem extends SubsystemBase {
    private final TalonFX m_motor32; // CAN ID 32 preserved

    // Original: kDegreesPerRotation = 360.0 / (4.0 * 6.5)
    // Gear ratio: 4.0 * 6.5 = 26 motor rotations per mechanism rotation.
    // With SensorToMechanismRatio = 26, position.getValueAsDouble() is in mechanism rotations.
    // Degrees = mechanism_rotations * 360.0
    private static final double kGearRatio = 4.0 * 6.5; // 26

    private final DutyCycleOut    m_dutyCycleRequest = new DutyCycleOut(0);
    private final PositionDutyCycle m_positionRequest = new PositionDutyCycle(0);

    public TurretSubsystem() {
        m_motor32 = new TalonFX(32);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Gear ratio: position signal will be in mechanism rotations
        config.Feedback.SensorToMechanismRatio = kGearRatio;

        // NOTE: Original kP = 0.18/degree. Converted to per-rotation: 0.18 * 360 = 64.8.
        // Re-tune for Kraken X60 on actual hardware.
        config.Slot0.kP = 64.8;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;

        // Output limits mirror the original -0.4 to 0.4 range
        config.MotorOutput.PeakForwardDutyCycle =  0.4;
        config.MotorOutput.PeakReverseDutyCycle = -0.4;

        // Soft limits in mechanism rotations (original: ±170 degrees -> ±170/360 rotations)
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =  170.0 / 360.0;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable    = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -170.0 / 360.0;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable    = true;

        // Reverse hardware limit switch for homing (stops motor in reverse direction when triggered)
        config.HardwareLimitSwitch.ReverseLimitEnable = true;

        m_motor32.getConfigurator().apply(config);
    }

     Returns true when the reverse (home) limit switch is triggered. 
    private boolean isAtHome() {
        return m_motor32.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
    }

     Returns the current turret angle in degrees. 
    private double getAngleDegrees() {
        return m_motor32.getPosition().getValueAsDouble() * 360.0;
    }

    public void setAngle(double degrees) {
        m_motor32.setControl(m_positionRequest.withPosition(degrees / 360.0));
    }

    public Command goToAngleCommand(double degrees) {
        return this.run(() -> setAngle(degrees));
    }

    /**
     * Improved Homing:
     * 1. Checks if already on switch. If so, reset and finish.
     * 2. If not, drive reverse (-0.05) until switch is hit.
     */
    /*public Command findHomeCommand() {
        return Commands.either(
            // If already pressed:
            this.runOnce(() -> {
                m_motor32.setPosition(0.0);
                System.out.println("Turret already home. Resetting encoder.");
            }),
            // If NOT pressed, run the movement sequence:
            this.run(() -> m_motor32.setControl(m_dutyCycleRequest.withOutput(-0.05)))
                .until(this::isAtHome)
                .finallyDo((interrupted) -> {
                    m_motor32.setControl(m_dutyCycleRequest.withOutput(0));
                    if (!interrupted) {
                        m_motor32.setPosition(0.0);
                        System.out.println("Turret homing complete.");
                    }
                }),
            this::isAtHome
        );
    }

    public void runManual(double speed) {
        m_motor32.setControl(m_dutyCycleRequest.withOutput(speed));
    }

    public void stop() {
        m_motor32.setControl(m_dutyCycleRequest.withOutput(0));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Turret/Angle",  getAngleDegrees());
        SmartDashboard.putBoolean("Turret/Home",  isAtHome());
    }
}
*/