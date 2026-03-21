package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import frc.robot.Constants.PivotConstants;

public class PivotSubsystem extends SubsystemBase {
    private final SparkMax m_motor = new SparkMax(PivotConstants.kPivotMotorID, MotorType.kBrushless);
    private final SparkClosedLoopController m_pid = m_motor.getClosedLoopController();
    private final RelativeEncoder m_encoder = m_motor.getEncoder();

    // 2x 5:1 = 25:1 gear ratio — degrees per motor rotation
    private static final double kDegreesPerRotation = 360.0 / 25.0;

    // TODO: Tune these positions (in degrees) on the actual robot
    public static final double kPositionUp   = 0.0;  // placeholder
    public static final double kPositionDown = 75; // placeholder

    // TODO: Tune PID gains on the actual robot
    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    // Speed used for manual raise/lower (bumpers)
    private static final double kRaiseSpeed = -0.3;
    private static final double kLowerSpeed =  0.3;

    public PivotSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(40);

        config.encoder
            .positionConversionFactor(kDegreesPerRotation)
            .velocityConversionFactor(kDegreesPerRotation / 60.0);

        config.closedLoop
            .p(kP)
            .i(kI)
            .d(kD)
            .outputRange(-1.0, 1.0);

        m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /** Go to a target angle in degrees and hold. */
    public void setPosition(double degrees) {
        m_pid.setSetpoint(degrees, ControlType.kPosition);
    }

    /** Returns current angle in degrees. */
    public double getPosition() {
        return m_encoder.getPosition();
    }

    /** Runs the pivot at the given duty cycle speed (-1.0 to 1.0). */
    public void setPivotSpeed(double speed) {
        m_motor.set(speed);
    }

    /** Stops the pivot motor. Brake mode will hold position. */
    public void stop() {
        m_motor.set(0);
    }

    /** Holds the "up" position while the button is held. */
    public Command goUpCommand() {
        return this.run(() -> setPosition(kPositionUp))
                   .withName("Pivot Up");
    }

    /** Holds the "down" position while the button is held. */
    public Command goDownCommand() {
        return this.run(() -> setPosition(kPositionDown))
                   .withName("Pivot Down");
    }

    /** Raises manually while held, stops on release. */
    public Command raiseCommand() {
        return this.startEnd(
            () -> setPivotSpeed(kRaiseSpeed),
            () -> stop()
        ).withName("Pivot Raise");
    }

    /** Lowers manually while held, stops on release. */
    public Command lowerCommand() {
        return this.startEnd(
            () -> setPivotSpeed(kLowerSpeed),
            () -> stop()
        ).withName("Pivot Lower");
    }
}
