package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * Autonomous command: turns the turret to center on an AprilTag.
 *
 * - Runs the turret proportionally toward tx = 0 (tag centered in camera).
 * - Finishes when aligned within 2 degrees OR after 3 seconds (timeout).
 * - Stops the turret motor when done.
 *
 * Register in RobotContainer with:
 *   NamedCommands.registerCommand("AlignTurretToTag", new AutoTurretAlign(m_turret, m_vision));
 * Then add a "AlignTurretToTag" named command step in PathPlanner GUI.
 */
public class AutoTurretAlign extends Command {

    private final TurretSubsystem m_turret;
    private final VisionSubsystem m_vision;

    // Proportional gain: 1 degree of error → 0.02 motor output
    // Increase if turret is too slow; decrease if it overshoots/oscillates
    private static final double kP = 0.02;

    // Command ends when tx is within this many degrees of center
    private static final double kToleranceDegrees = 2.0;

    // Safety timeout — command will end even if no tag is ever seen
    private static final double kTimeoutSeconds = 3.0;

    private final Timer m_timer = new Timer();

    public AutoTurretAlign(TurretSubsystem turret, VisionSubsystem vision) {
        m_turret = turret;
        m_vision = vision;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
        System.out.println("[AutoTurretAlign] Started");
    }

    @Override
    public void execute() {
        if (!m_vision.hasTarget()) {
            // No tag visible yet — hold still and wait
            m_turret.runManual(0);
            return;
        }

        double tx = m_vision.getTX();

        // Proportional output — negative because positive tx means tag is to the RIGHT,
        // so we need to spin the turret right (positive) to chase it.
        // Flip the sign here if the turret moves the wrong way.
        double speed = tx * kP;

        // Clamp to safe range
        speed = Math.max(-0.4, Math.min(0.4, speed));

        m_turret.runManual(speed);
        System.out.println("[AutoTurretAlign] tx=" + tx + " speed=" + speed);
    }

    @Override
    public boolean isFinished() {
        if (m_timer.hasElapsed(kTimeoutSeconds)) {
            System.out.println("[AutoTurretAlign] Timed out");
            return true;
        }

        boolean aligned = m_vision.hasTarget()
                && Math.abs(m_vision.getTX()) < kToleranceDegrees;

        if (aligned) {
            System.out.println("[AutoTurretAlign] Aligned!");
        }
        return aligned;
    }

    @Override
    public void end(boolean interrupted) {
        m_turret.runManual(0);
        System.out.println("[AutoTurretAlign] Ended (interrupted=" + interrupted + ")");
    }
}
