package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralPivotSubsystem;

public class CoralUpCommand extends Command {

    private final CoralPivotSubsystem m_coralPosition;

    // Make code raise to scoring angle

    public CoralUpCommand(CoralPivotSubsystem coralPosition) {

        m_coralPosition = coralPosition;
        addRequirements(m_coralPosition);

    }

    // NOT USED RIGHT NOW

    @Override
    public void initialize() {

        m_coralPosition.pivotUp();
        //m_coralPosition.SetPivotRaised();
    }

    @Override
    public boolean isFinished() {

        return true;

    }

}