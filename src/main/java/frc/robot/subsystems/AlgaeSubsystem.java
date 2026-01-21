package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants.AlgaeSubsystemConstants;

// motor 61
public class AlgaeSubsystem extends SubsystemBase {

    private SparkMax algae;

    public AlgaeSubsystem(int deviceId) {
     //Dino 61 (we think)
      algae = new SparkMax(deviceId, MotorType.kBrushless);

    }

    public void IntakeAlgae() {

        algae.set(AlgaeSubsystemConstants.k_intakeFactor);

    }

    public void IntakeAlgae(double speed) {
        speed = -(Math.abs(speed));
        algae.set(speed);
    }

    public void ReleaseAlgae() {

        algae.set(AlgaeSubsystemConstants.k_releaseFactor);

    }

    public void ReleaseAlgae(double speed) {

        speed = Math.abs(speed);
        algae.set(speed);

    }

    public void Still() {

        algae.set(0);

    }
}