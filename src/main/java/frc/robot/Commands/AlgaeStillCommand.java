package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;

public class AlgaeStillCommand extends Command {

    private final AlgaeSubsystem m_algae;

    public AlgaeStillCommand(AlgaeSubsystem subsystem) {

        m_algae = subsystem;
        addRequirements(m_algae);

    }

    @Override
    public void initialize() {
        m_algae.Still();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}