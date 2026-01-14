package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants.CoralSubsystemConstants;

// motor 21
public class CoralSubsystem extends SubsystemBase {

    private SparkMax coral;
    private boolean m_lowered;

    public CoralSubsystem(int deviceId) {

        coral = new SparkMax(deviceId, MotorType.kBrushless);

    }

    public void ElevatorIsLowered(boolean lowered) {
        m_lowered = lowered;
    }

    public void IntakeCoral() {

        coral.set(CoralSubsystemConstants.k_intakeFactor);
    }

    public void Intakecoral(double speed) {
        speed = -(Math.abs(speed));
        coral.set(speed);
    }

    public void ReleaseCoral() {

        if (m_lowered) {
            coral.set(CoralSubsystemConstants.k_releaseFactorLow);
        } else {
            coral.set(CoralSubsystemConstants.k_releaseFactor);
        }

    }

    public void ReleaseCoral(double speed) {

        speed = Math.abs(speed);

        coral.set(speed);

    }

    public void Still() {

        coral.set(0);

    }
}