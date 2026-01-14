
package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralPivotSubsystem;

public class CoralPositionStillCommand extends Command {

    private final CoralPivotSubsystem m_coralPosition;

    public CoralPositionStillCommand(CoralPivotSubsystem coralPosition) {

        m_coralPosition = coralPosition;
        addRequirements(m_coralPosition);

    }

    @Override
    public void initialize() {

        m_coralPosition.pivotStill();

    }

    @Override
    public boolean isFinished() {

        return true;

    }
}
