package frc.robot.subsystems;

//import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLimitSwitch;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants.CoralPivotSubsystemConstants;

// motor 22
public class CoralPivotSubsystem extends SubsystemBase {

    private SparkMax m_coralPivot;
    private RelativeEncoder m_relativeEncoder;
    private SparkLimitSwitch m_limitSwitch;
    private boolean m_findHome = false;
    private boolean m_findLow = false;
    private boolean m_goingUp;
    private boolean m_goingDown;

    // private ShuffleboardTab coralTab = Shuffleboard.getTab("Coral");

    public CoralPivotSubsystem(int deviceId) {

     //Dino   m_coralPivot = new SparkMax(deviceId, MotorType.kBrushless);
        m_relativeEncoder = m_coralPivot.getEncoder();
        m_limitSwitch = m_coralPivot.getForwardLimitSwitch();

        // coralTab.addNumber("Coral Position", () -> getPivotEncoder());
        // coralTab.addBoolean("Coral Raised", () -> isRaised());
        // coralTab.addBoolean("Coral Lowered", () -> isLowered());

        SetZeroInit();

    }

    public boolean goingUp(){
        return m_goingUp;
    }
    public boolean goingDown(){
        return m_goingDown;
    }

    public void SetZeroInit() {

        m_findHome = true;

        if (isRaised() == false) {
            pivotUp();

        } else {
            m_findHome = false;
        }
    }

    private void SetZeroFinish() {

        pivotUp();

        if (isRaised()) {
            pivotStill();
            setZero();

            m_findHome = false;
            SetLowInit();

        }
    }

    public void SetLowInit() {

        m_findLow = true;

        if (isLowered() == false) {
            pivotDown();

        } else {
            m_findLow = false;
        }
    }

    private void SetLowFinish() {

        pivotDown();

        if (isLowered()) {
            pivotStill();

            m_findLow = false;
        }
    }

    public double getPosition() {

        return m_relativeEncoder.getPosition();

    }

    public void setZero() {

        m_relativeEncoder.setPosition(0);

    }

    public void pivotUp() {

        if (isRaised()) {
            pivotStill();
            m_relativeEncoder.setPosition(0);
        } else {
          //Dino  m_coralPivot.set((CoralPivotSubsystemConstants.k_speedUpFactor));
            m_goingUp = true;
            m_goingDown = false;
        }

    }

    public void pivotUp(double speed) {
        speed = -(Math.abs(speed));

        if (isRaised()) {
            pivotStill();
            m_relativeEncoder.setPosition(0);
        } else {
            m_coralPivot.set((speed));
            m_goingUp = true;
            m_goingDown = false;
        }

    }

    public void pivotDown() {

        m_goingDown = true;

        if (isLowered()) {
            pivotStill();
        } else {
            m_coralPivot.set(CoralPivotSubsystemConstants.k_speedDownFactor);
            m_goingUp = false;
            m_goingDown = true;
        }

    }

    public void pivotDown(double speed) {
        speed = Math.abs(speed);

        if (isLowered()) {
            pivotStill();
        } else {
            m_coralPivot.set(speed);
            m_goingUp = false;
            m_goingDown = true;
        }

    }

    public void pivotUpInit() {

        m_coralPivot.set(CoralPivotSubsystemConstants.k_speedUpFactor);

    }

    public void pivotStill() {

        m_coralPivot.set(0);
        m_goingUp = false;
        m_goingDown = false;

        if (isRaised()) {
            m_relativeEncoder.setPosition(0);
        }

    }

    public boolean isRaised() {

        return m_limitSwitch.isPressed();

    }

    public boolean isLowered() {

        return (Math.abs(CoralPivotSubsystemConstants.k_pointLowered - getPosition()) <= 1.5);

    }

    public void SetPivotRaised() {
        m_goingUp = true;
        m_goingDown = false;

        if (isRaised()) {
            pivotStill();
        } else { 
            pivotUp();
        }

    }

    public void SetPivotLowered() {
        m_goingUp = false;
        m_goingDown = true;

        if (isLowered()) {
            pivotStill();
        } else { 
            pivotDown();
        }

    }

    private void UpdateDashboard() {

        SmartDashboard.putNumber("Coral Position", getPosition());
        SmartDashboard.putBoolean("Coral Raised", isRaised());
        SmartDashboard.putBoolean("Coral Lowered", isLowered());
        SmartDashboard.putBoolean("Coral Find Home", m_findHome);
        SmartDashboard.putBoolean("Coral Find Low", m_findLow);
    }

    public void periodic() {

        if (m_findHome) {
            SetZeroFinish();
        }

        if (m_findLow) {
            SetLowFinish();
        }

        UpdateDashboard();

        if (isRaised()) {
            if (m_goingUp) {
                pivotStill();
            }
        }
        if (isLowered()) {
            if (m_goingDown) {
                pivotStill();
            }
        }
    }

}