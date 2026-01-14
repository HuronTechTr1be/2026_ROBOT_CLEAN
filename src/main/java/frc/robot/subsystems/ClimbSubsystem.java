package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLimitSwitch;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants.ClimbSubsystemConstants;

// motor 51
public class ClimbSubsystem extends SubsystemBase {

    private ClimbPivotSubsystem m_climbLeft;
    private ClimbPivotSubsystem m_climbRight;
    private ClimbWinchSubsystem m_climbWinch;
    private boolean m_findHome;
    private boolean m_movingDown = false;
    private boolean m_movingUp = false;
    private String m_target = new String();


    public ClimbSubsystem() {

//        m_climbLeft = new ClimbPivotSubsystem(51, "left");
//        m_climbRight = new ClimbPivotSubsystem(52, "right");
      //Dino m_climbWinch = new ClimbWinchSubsystem(51);
//        SetZeroInit();

    }


    // public void SetZeroInit() {

    //     m_findHome = true;
    //     m_climbLeft.setZero();
    //     m_climbRight.setZero();

    //     if (atUpperLimit() == false) {
    //         ClimbUp(ClimbSubsystemConstants.k_speedUpFactor);
    //     } else {
    //         m_findHome = false;
    //     }
    // }

    // private void SetZeroFinish() {

    //     ClimbUp();

    //     if (atUpperLimit()) {
    //         ClimbStill();
    //         m_climbLeft.setZero();
    //         m_climbRight.setZero();

    //         m_findHome = false;
    //     }
    // }

    private void UpdateDashboard() {

        // SmartDashboard.putBoolean("Climb Find Home", m_findHome);
        // SmartDashboard.putBoolean("Climb Lowered", atLowerLimit());
        // SmartDashboard.putBoolean("Climb Raised", atUpperLimit());
        // SmartDashboard.putNumber("Climb Left Position", m_climbLeft.getPosition());
        // SmartDashboard.putNumber("Climb Right Position", m_climbRight.getPosition());
        SmartDashboard.putNumber("Climb Winch Position", getPosition());
    }

    public void periodic() {

        // if (m_findHome) {
        //     SetZeroFinish();
        // }

        // if (ClimbLowered()) {
        //     if (m_target == "Lowered") {
        //         ClimbStill();
        //     }
        // }

        // if (ClimbMiddle()) {
        //     if (m_target == "Middle") {
        //         ClimbStill();
        //     }
        // }

        // if (ClimbRaised()) {
        //     m_climbLeft.setZero();
        //     m_climbRight.setZero();
        //     if (m_target == "Raised") {
        //         ClimbStill();
        //     }
        // }

        // m_climbRight.periodic();
        // m_climbLeft.periodic();

        UpdateDashboard();

    }

    public double getPosition() {
        return m_climbWinch.getPosition();
    }

    public boolean atLowerLimit() {
        return false;   //m_climbLeft.isOnSwitch();
    }

    public boolean atUpperLimit() {
        return false;   //m_climbRight.isOnSwitch();
    }


    public boolean ClimbLowered() {
        return atLowerLimit();
    }

    public boolean ClimbRaised() {
        return atUpperLimit();
    }

    public boolean ClimbMiddle() {
        return (Math.abs(getPosition() - ClimbSubsystemConstants.k_PointMiddle) < 2);
    }


    public void ClimbDown(double speed) {

        m_movingDown = true;

        if (ClimbLowered()) {
            // m_climbLeft.Still();
            // m_climbRight.Still();
            m_climbWinch.Still();
        } else {
            // m_climbLeft.Down(speed);
            // m_climbRight.Down(speed);
            m_climbWinch.Down(speed);
            m_movingDown = true;

        }

    }

    public void ClimbDown() {

        m_movingDown = true;

        if (ClimbLowered()) {
            // m_climbLeft.Still();
            // m_climbRight.Still();
            m_climbWinch.Still();
        } else {
            // m_climbLeft.Down();
            // m_climbRight.Down();
            m_climbWinch.Down();
            m_movingDown = true;

        }

    }

    
    public void ClimbUp(double speed) {
        if (ClimbRaised()) {
            // m_climbLeft.Still();
            // m_climbRight.Still();
            m_climbWinch.Still();
        } else {
            // m_climbLeft.Up(speed);
            // m_climbRight.Up(speed);
            m_climbWinch.Up(speed);
            m_movingUp = true;

        }

    }
    
    public void ClimbUp() {
        if (ClimbRaised()) {
            // m_climbLeft.Still();
            // m_climbRight.Still();
            m_climbWinch.Still();
        } else {
            // m_climbLeft.Up();
            // m_climbRight.Up();
            m_climbWinch.Up();
            m_movingUp = true;

        }

    }

    public void ClimbStill() {
        // m_climbLeft.Still();
        // m_climbRight.Still();
        m_climbWinch.Still();

        m_target = "";
        m_movingDown = false;
        m_movingUp = false;

    }


    public void SetClimbRaised() {
        m_target = "Raised";
        if (ClimbRaised()) {
            ClimbStill();
        } else { // if (getPosition() < ElevatorSubsystemConstants.m_PointRaised) {
            ClimbUp();
        }
    }

    public void SetClimbMiddle() {
        m_target = "Middle";
        if (ClimbMiddle()) {
            ClimbStill();
        } else if (getPosition() < ClimbSubsystemConstants.k_PointMiddle) {
            ClimbUp();
        } else if (getPosition() > ClimbSubsystemConstants.k_PointMiddle) {
            ClimbDown();
        }
    }

    public void SetClimbLowered() {
        m_target = "Lowered";
        if (ClimbLowered()) {
            ClimbStill();
        } else {
            ClimbDown();
        }
    }

}