
package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbStillCommand extends Command {

    private final ClimbSubsystem m_climb;

    public ClimbStillCommand(ClimbSubsystem climb) {

        m_climb = climb;
        addRequirements(m_climb);

    }

    @Override
    public void initialize() {

        m_climb.ClimbStill();

    }

    @Override
    public boolean isFinished() {

        return true;

    }
}
