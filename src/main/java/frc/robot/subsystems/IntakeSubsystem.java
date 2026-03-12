package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class IntakeSubsystem extends SubsystemBase {
    // Define the two motors. REPLACE 90 and 91 with your actual CAN IDs.  DONE
    private final SparkMax m_frontRoller = new SparkMax(10, MotorType.kBrushless);
    private final SparkMax m_backRoller = new SparkMax(11, MotorType.kBrushless);
    private final SparkMax m_Roller = new SparkMax(12, MotorType.kBrushless);

    public IntakeSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kCoast); 
        config.smartCurrentLimit(60);    
        
        // Apply config to both motors
        m_frontRoller.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_backRoller.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_Roller.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    /**
     * Run both intake rollers at independent speeds.
     * @param frontSpeed Speed for the front roller (-1.0 to 1.0)
     * @param backSpeed Speed for the back roller (-1.0 to 1.0)
     */
    public Command runIntakeCommand(double frontSpeed, double backSpeed) {
        return this.startEnd(
            () -> {
                m_frontRoller.set(frontSpeed);
                m_backRoller.set(backSpeed);
                m_Roller.set(frontSpeed);
            }, 
            () -> {
                m_frontRoller.set(0);
                m_backRoller.set(0);
                m_Roller.set(0);
            }
        ).withName("Run Intake Dual");
    }
}