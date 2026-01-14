
package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorSetMiddleCommand extends Command {

    private final ElevatorSubsystem m_elevator;

    public ElevatorSetMiddleCommand(ElevatorSubsystem elevator) {

        m_elevator = elevator;
        addRequirements(m_elevator);

    }

    @Override
    public void initialize() {

        m_elevator.SetElevatorMiddle();

    }

    @Override
    public boolean isFinished() {

        return true;

    }

}
