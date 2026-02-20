package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode.
   * Use this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   */
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
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }
}