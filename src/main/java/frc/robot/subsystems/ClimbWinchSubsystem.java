package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generated.TunerConstants.ClimbSubsystemConstants;

public class ClimbWinchSubsystem {
    private SparkMax m_climb;
    private RelativeEncoder m_RelativeEncoder;

    private SparkLimitSwitch m_LimitSwitch;
    public String m_target = "";

    public ClimbWinchSubsystem(int deviceId) {

       //Dino m_climb = new SparkMax(deviceId, MotorType.kBrushless);
        m_RelativeEncoder = m_climb.getEncoder();
        m_LimitSwitch = m_climb.getForwardLimitSwitch();

    }

    public void Up() {

        m_climb.set(ClimbSubsystemConstants.k_speedUpFactor);
    }

    public void Up(double speed) {

        m_climb.set((speed));
    }

    public void Down() {
        m_climb.set(ClimbSubsystemConstants.k_speedDownFactor);
    }

    public void Down(double speed) {
        m_climb.set(speed);
    }

    public void Still() {
        m_climb.set(0);
        m_target = "";
    }


    public boolean isOnSwitch() {
        return false;   // (m_LimitSwitch.isPressed());
    }

    public double getCurrent() {
        double ClimbCurrent = m_climb.getOutputCurrent();
        SmartDashboard.putNumber("ClimbWinch Current", ClimbCurrent);
        return ClimbCurrent;
    }

    public void setZero() {
        m_RelativeEncoder.setPosition(0);
    }

    public double getPosition() {
        return m_RelativeEncoder.getPosition();
    }

    public void periodic() {

    }

}
