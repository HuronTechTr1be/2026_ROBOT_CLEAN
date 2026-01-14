package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;

public class CoralReleaseCommand extends Command {

    private final CoralSubsystem m_coral;

    public CoralReleaseCommand(CoralSubsystem subsystem) {

        m_coral = subsystem;
        addRequirements(m_coral);

    }

    @Override
    public void initialize() {
        m_coral.ReleaseCoral();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}