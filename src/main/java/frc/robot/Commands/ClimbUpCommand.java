
package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbUpCommand extends Command {

    private final ClimbSubsystem m_climb;

    public ClimbUpCommand(ClimbSubsystem climb) {

        m_climb = climb;
        addRequirements(m_climb);

    }

    // NOT USED RIGHT NOW

    @Override
    public void initialize() {

        m_climb.ClimbUp();

    }

    @Override
    public boolean isFinished() {

        return true;

    }

}
