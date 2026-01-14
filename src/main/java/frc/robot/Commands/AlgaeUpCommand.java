
package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaePivotSubsystem;

public class AlgaeUpCommand extends Command {

    private final AlgaePivotSubsystem m_algaePosition;

    public AlgaeUpCommand(AlgaePivotSubsystem algaePosition) {

        m_algaePosition = algaePosition;
        addRequirements(m_algaePosition);

    }

    // NOT USED RIGHT NOW

    @Override
    public void initialize() {

        m_algaePosition.algaePivotUp();

    }

    @Override
    public boolean isFinished() {

        return true;

    }

}
