package frc.robot.subsystems;

import com.revrobotics.spark.SparkLimitSwitch;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLimitSwitch;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants.AlgaePivotSubsystemConstants;
import frc.robot.generated.TunerConstants.ElevatorSubsystemConstants;

// motor 62
public class AlgaePivotSubsystem extends SubsystemBase {

    private SparkMax m_algaePivot;
    private RelativeEncoder m_relativeEncoder;
    private SparkLimitSwitch m_limitSwitch;
    private boolean m_findHome = false;
    private boolean m_goingUp;
    private boolean m_goingDown;
    private String m_target = new String();

    public AlgaePivotSubsystem(int deviceId) {

      //Dino  m_algaePivot = new SparkMax(deviceId, MotorType.kBrushless);
        m_relativeEncoder = m_algaePivot.getEncoder();
        m_limitSwitch = m_algaePivot.getForwardLimitSwitch();

        SetZeroInit();
    }

    public void SetZeroInit() {

        m_findHome = true;
        //m_relativeEncoder.setPosition(100);

        if (isRaised() == false) {
            algaePivotUp();

        } else {
            m_findHome = false;
        }
    }

    private void SetZeroFinish() {

        algaePivotUp();

        if (isRaised() == true) {
            algaePivotStill();
            setZero();

            m_findHome = false;

        }
    }

    public double getPosition() {

        return m_relativeEncoder.getPosition();

    }

    public void setZero() {

        m_relativeEncoder.setPosition(0);

    }

    public void algaePivotUp() {

        m_goingUp = true;

        if (isRaised()) {
            algaePivotStill();
        } else {
            m_algaePivot.set(AlgaePivotSubsystemConstants.k_speedUpFactor);
            m_goingDown = false;
            m_goingUp = true;
        }

    }

    public void algaePivotUp(double speed) {
        // speed = (Math.abs(speed));
        // if (onSwitch()) {
        // if (goingDown == false){
        // algaePivot.set(0);
        // m_relativeEncoder.setPosition(0);
        // goingUp = true;
        // }
        // else{
        // algaePivot.set(speed);
        // }
        // } else {
        // algaePivot.set((speed));
        // }

    }

    public void algaePivotDown() {

        m_goingDown = true;

        if (isLowered()) {
            algaePivotStill();
        } else {
            m_algaePivot.set(AlgaePivotSubsystemConstants.k_speedDownFactor);
            m_goingDown = true;
            m_goingUp = false;
        }

    }

    public void algaePivotDown(double speed) {
        // speed = -(Math.abs(speed));
        // if (onSwitch()) {
        // if (goingUp == false){
        // algaePivot.set(0);
        // goingDown = true;
        // }
        // else{
        // algaePivot.set(speed);
        // }
        // } else {
        // algaePivot.set(speed);
        // }
    }

    public void algaePivotStill() {

        m_algaePivot.set(0);

        m_target = "";
        m_goingUp = false;
        m_goingDown = false;

    }

    public boolean AlgaePivotMiddle() {
        return (Math.abs(getPosition() - AlgaePivotSubsystemConstants.k_pointMiddle) < 2);
    }

    public boolean AlgaePivotLowered() {
        return (Math.abs(getPosition() - AlgaePivotSubsystemConstants.k_pointLowered) < 2);
    }

    public void SetAlgaePivotMiddle() {
        m_target = "Middle";
        if (AlgaePivotMiddle()) {
            algaePivotStill();
        } else if (getPosition() < AlgaePivotSubsystemConstants.k_pointMiddle) {
            algaePivotDown();
        } else if (getPosition() > AlgaePivotSubsystemConstants.k_pointMiddle) {
            algaePivotUp();
        }
    }

    public void SetAlgaePivotLowered() {
        m_target = "Lowered";
        if (AlgaePivotLowered()) {
            algaePivotStill();
        } else if (getPosition() < AlgaePivotSubsystemConstants.k_pointLowered) {
            algaePivotDown();
        } else if (getPosition() > AlgaePivotSubsystemConstants.k_pointLowered) {
            algaePivotUp();
        }
    }

    public boolean onSwitch() {

        return m_limitSwitch.isPressed();

    }

    public boolean isRaised() {

        return (m_limitSwitch.isPressed()); // || (getPosition() < 0));

    }

    public boolean isLowered() {

        return ((AlgaePivotSubsystemConstants.k_pointLowered - getPosition()) <= 2);

    }

    private void UpdateDashboard() {

        SmartDashboard.putNumber("Algae Position", getPosition());
        SmartDashboard.putBoolean("Algae Raised", isRaised());
        SmartDashboard.putBoolean("Algae Middle", AlgaePivotMiddle());
        SmartDashboard.putBoolean("Algae Lowered", isLowered());
        SmartDashboard.putBoolean("Algae GoingUp", m_goingUp);
        SmartDashboard.putBoolean("Algae GoingDown", m_goingDown);
    }

    public void periodic() {

        if (m_findHome) {
            SetZeroFinish();
        }

        UpdateDashboard();

        if (isRaised()) {
            if (m_goingUp) {
                algaePivotStill();
            }
        }
        if (AlgaePivotMiddle()) {
            if (m_target == "Middle") {
                algaePivotStill();
            }
        }
        if (isLowered()) {
            if (m_goingDown) {
                algaePivotStill();
            }
        }

    }

}