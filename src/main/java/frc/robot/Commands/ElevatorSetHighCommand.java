
package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorSetHighCommand extends Command {

    private final ElevatorSubsystem m_elevator;

    public ElevatorSetHighCommand(ElevatorSubsystem elevator) {

        m_elevator = elevator;
        addRequirements(m_elevator);

    }

    @Override
    public void initialize() {

        m_elevator.SetElevatorHigh();

    }

    @Override
    public boolean isFinished() {

        return true;

    }

}
