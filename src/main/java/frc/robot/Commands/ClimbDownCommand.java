
package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbDownCommand extends Command {

    private final ClimbSubsystem m_climb;

    public ClimbDownCommand(ClimbSubsystem climb) {

        m_climb = climb;
        addRequirements(m_climb);

    }

    @Override
    public void initialize() {

        m_climb.ClimbDown();

    }

    @Override
    public boolean isFinished() {

        return true;

    }

}
