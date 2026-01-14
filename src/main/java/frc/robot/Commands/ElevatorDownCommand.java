
package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorDownCommand extends Command {

    private final ElevatorSubsystem m_elevator;

    public ElevatorDownCommand(ElevatorSubsystem elevator) {

        m_elevator = elevator;
        addRequirements(m_elevator);

    }

    @Override
    public void initialize() {

        m_elevator.ElevatorDown();

    }

    @Override
    public boolean isFinished() {

        // m_elevator.ElevatorStill();
        return true;

    }

}
