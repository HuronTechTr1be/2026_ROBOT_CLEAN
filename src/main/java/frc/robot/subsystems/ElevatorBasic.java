package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLimitSwitch;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generated.TunerConstants.ElevatorSubsystemConstants;

//NEED TO TEST VALUES AND CHANGE CONSTANTS

//start: 31 = -349 & 32 = -438
// isRaised: 31 = -433 & 32 = -521 (went about 90)

public class ElevatorBasic {
    private SparkMax elevator;
    private RelativeEncoder m_RelativeEncoder;

    // Test PIDController Code
    // private PIDController pid;
    // private double targetPosition = 50.0;

    private String m_elevatorSide;
    private SparkLimitSwitch m_LimitSwitch;
    public String target = "";

    public ElevatorBasic(int deviceId, String elevatorSide) {

      //Dino  elevator = new SparkMax(deviceId, MotorType.kBrushless);
        m_RelativeEncoder = elevator.getEncoder();
        m_LimitSwitch = elevator.getForwardLimitSwitch();

        m_elevatorSide = elevatorSide;

        // pid = new PIDController(2, 0, 0);

    }

    // public void setRampRate(int rampRate) {

    //     elevator.setRampRate(rampRate);

    // }

    // Test PIDController Code
    // public void setTargetPosition(double position){
    // targetPosition = position;
    // pid.setSetpoint(targetPosition);
    // }

    // Test PIDController Code
    // public double getTargetPosition() {
    // return targetPosition;
    // }

    public void Up() {

        elevator.set(ElevatorSubsystemConstants.k_ElevatorSpeedUp);
    }

    public void Up(double speed) {

        elevator.set((speed));
    }

    public void Down() {
        elevator.set(ElevatorSubsystemConstants.k_ElevatorSpeedDown);
    }

    public void Down(double speed) {
        elevator.set(speed);
    }

    public void Still() {
        elevator.set(0);
        target = "";
    }

    // public boolean isRaised() {

    // // Don't know if 5 is a reasonable range here, need to test encoders and get
    // a
    // // sense of scale

    // boolean raised = Math.abs(ElevatorSubsystemConstants.k_PointRaised -
    // getPosition()) <= 10;
    // SmartDashboard.putBoolean(m_elevatorSide + "raised", raised);

    // return raised;
    // }

    // public boolean isMiddle() {
    // return Math.abs(ElevatorSubsystemConstants.k_PointMiddle - getPosition()) <=
    // 10;
    // }

    // public boolean isLow() {
    // return Math.abs(ElevatorSubsystemConstants.k_PointLow - getPosition()) <= 10;
    // }

    // public boolean isLowered() {
    // return Math.abs(ElevatorSubsystemConstants.k_PointLow - getPosition()) <= 10;
    // }

    public boolean isOnSwitch() {
        return (m_LimitSwitch.isPressed());
    }

    public double getCurrent() {
        double ElevatorCurrent = elevator.getOutputCurrent();
        SmartDashboard.putNumber(m_elevatorSide + "ArmCurrent", ElevatorCurrent);
        return ElevatorCurrent;
    }

    public void setZero() {
        m_RelativeEncoder.setPosition(0);
    }

    // public void finishZero() {

    // m_RelativeEncoder.setPosition(0);
    // elevator.setOpenLoopRampRate(0);
    // elevator.burnFlash();

    // }

    public double getPosition() {
        return m_RelativeEncoder.getPosition();
    }

    public void periodic() {

        // Test PIDController Code
        // elevator.set(pid.calculate(getPosition()));

    }

}