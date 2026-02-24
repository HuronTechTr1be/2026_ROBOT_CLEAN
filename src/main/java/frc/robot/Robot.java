package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.util.PixelFormat;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
//line 14-35 camera
  @Override
  public void robotInit() {
    // 1. Initialize the RobotContainer
    m_robotContainer = new RobotContainer();

    // 2. Start the USB Camera on port 0
    UsbCamera camera = CameraServer.startAutomaticCapture(0);
    camera.setVideoMode(PixelFormat.kMJPEG, 320, 240, 15);
    camera.setBrightness(50);
  }

  @Override
  public void robotPeriodic() {
    // 1. Run the Scheduler (updates all Subsystems)
    CommandScheduler.getInstance().run(); 

    // 2. Update the Dashboard (Field2d, etc.)
    if (m_robotContainer != null) {
        // I ADDED THE TWO SLASHES HERE TO FIX THE ERROR:
        // m_robotContainer.updateDashboard(); 
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when teleop starts running.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }
}


/*package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.util.PixelFormat;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    // 1. Initialize the RobotContainer
    m_robotContainer = new RobotContainer();

    // 2. Start the USB Camera on port 0
    UsbCamera camera = CameraServer.startAutomaticCapture(0);
    camera.setVideoMode(PixelFormat.kMJPEG, 320, 240, 15);
    camera.setBrightness(50);
  }

  @Override
  public void robotPeriodic() {
    // 1. Run the Scheduler (updates all Subsystems)
    CommandScheduler.getInstance().run(); 

    // 2. Update the Dashboard (Field2d, etc.)
    if (m_robotContainer != null) {
        m_robotContainer.updateDashboard();
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when teleop starts running.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }
}*/ 