package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants.ElevatorSubsystemConstants;

public class ElevatorSubsystem extends SubsystemBase {

    // Need to clarify which side is which and make clear what we're defining as
    // left and right
    private ElevatorBasic m_elevatorLeft;
    private ElevatorBasic m_elevatorRight;
    private boolean m_findHome;
    private boolean m_movingDown = false;
    private boolean m_movingUp = false;
    private String m_target = new String();

    public ElevatorSubsystem() {

        m_elevatorLeft = new ElevatorBasic(31, "left");
        m_elevatorRight = new ElevatorBasic(32, "right");
        SetZeroInit();
    }

    public void SetZeroInit() {

        m_findHome = true;
        m_elevatorLeft.setZero();
        m_elevatorRight.setZero();

        if (atLowerLimit() == false) {
            ElevatorDown(ElevatorSubsystemConstants.k_ElevatorSpeedDownSlow);
        } else {
            m_findHome = false;
        }
    }

    private void SetZeroFinish() {

        ElevatorDown();

        if (atLowerLimit()) {
            ElevatorStill();
            m_elevatorLeft.setZero();
            m_elevatorRight.setZero();

            m_findHome = false;
        }
    }

    private void UpdateDashboard() {

        SmartDashboard.putBoolean("Elevator Find Home", m_findHome);
        SmartDashboard.putBoolean("Elevator Lowered", atLowerLimit());
        SmartDashboard.putBoolean("Elevator Raised", atUpperLimit());
        SmartDashboard.putNumber("Elevator Left Position", m_elevatorLeft.getPosition());
        SmartDashboard.putNumber("Elevator Right Position", m_elevatorRight.getPosition());
    }

    public void periodic() {

        if (m_findHome) {
            SetZeroFinish();
        }

        if (ElevatorLowered()) {
            m_elevatorLeft.setZero();
            m_elevatorRight.setZero();
            if (m_target == "Lowered") {
                ElevatorStill();
            }
        }

        if (ElevatorLow()) {
            if (m_target == "Low") {
                ElevatorStill();
            }
        }
        if (ElevatorMiddle()) {
            if (m_target == "Middle") {
                ElevatorStill();
            }
        }
        if (ElevatorRaised()) {
            if (m_target == "High") {
                ElevatorStill();
            }
        }

        // redundant now I think - was this causing the jumping on and off issue?
        // if ((atLowerLimit() && m_movingUp == false) || (atUpperLimit() &&
        // m_movingDown == false)) {
        // ElevatorStill();
        // }

        m_elevatorRight.periodic();
        m_elevatorLeft.periodic();

        UpdateDashboard();

    }

    public double getPosition() {
        return m_elevatorRight.getPosition();
    }

    public boolean atLowerLimit() {
        return m_elevatorLeft.isOnSwitch();
    }

    public boolean atUpperLimit() {
        //return (ElevatorSubsystemConstants.k_PointRaised-m_elevatorRight.getPosition()<2);
        return m_elevatorRight.isOnSwitch();
    }

    public boolean ElevatorLowered() {
        return atLowerLimit();
    }

    public boolean ElevatorRaised() {
        return (atUpperLimit() || (getPosition() > ElevatorSubsystemConstants.k_PointRaised));
    }

    public boolean ElevatorMiddle() {
        return (Math.abs(getPosition() - ElevatorSubsystemConstants.k_PointMiddle) < 3);
    }

    public boolean ElevatorLow() {
        return (Math.abs(getPosition() - ElevatorSubsystemConstants.k_PointLow) < 3);
    }

    public void ElevatorDown(double speed) {

        m_movingDown = true;

        if (ElevatorLowered()) {
            m_elevatorLeft.Still();
            m_elevatorRight.Still();
        } else {
            m_elevatorLeft.Down(speed);
            m_elevatorRight.Down(speed);
            m_movingDown = true;

        }

    }

    public void ElevatorDown() {

        m_movingDown = true;

        if (ElevatorLowered()) {
            m_elevatorLeft.Still();
            m_elevatorRight.Still();
        } else {
            m_elevatorLeft.Down();
            m_elevatorRight.Down();
            m_movingDown = true;

        }

    }

    public void ElevatorUp(double speed) {

        if (ElevatorRaised()) {
            m_elevatorLeft.Still();
            m_elevatorRight.Still();
        } else {
            m_elevatorLeft.Up(speed);
            m_elevatorRight.Up(speed);
            m_movingUp = true;

        }

    }

    public void ElevatorUp() {
        if (ElevatorRaised()) {
            m_elevatorLeft.Still();
            m_elevatorRight.Still();
        } else {
            m_elevatorLeft.Up();
            m_elevatorRight.Up();
            m_movingUp = true;

        }

    }

    public void ElevatorStill() {
        m_elevatorLeft.Still();
        m_elevatorRight.Still();

        m_target = "";
        m_movingDown = false;
        m_movingUp = false;

    }

    public void SetElevatorHigh() {
        m_target = "High";
        if (ElevatorRaised()) {
            ElevatorStill();
        } else { // if (getPosition() < ElevatorSubsystemConstants.m_PointRaised) {
            ElevatorUp();
        }
    }

    public void SetElevatorMiddle() {
        m_target = "Middle";
        if (ElevatorMiddle()) {
            ElevatorStill();
        } else if (getPosition() < ElevatorSubsystemConstants.k_PointMiddle) {
            ElevatorUp();
        } else if (getPosition() > ElevatorSubsystemConstants.k_PointMiddle) {
            ElevatorDown();
        }
    }

    public void SetElevatorLow() {
        m_target = "Low";
        if (ElevatorLow()) {
            ElevatorStill();
        } else if (getPosition() < ElevatorSubsystemConstants.k_PointLow) {
            ElevatorUp();
        } else if (getPosition() > ElevatorSubsystemConstants.k_PointLow) {
            ElevatorDown();
        }
    }

    public void SetElevatorLowered() {
        m_target = "Lowered";
        if (ElevatorLowered()) {
            ElevatorStill();
        } else {
            ElevatorDown();
        }
    }

}