package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.util.Constants;

public class LockElevator extends Command {
    
    private Elevator elevator;

    public LockElevator (Elevator elevator) {

        this.elevator = elevator;
        this.addRequirements(this.elevator);
    }

    @Override
    public void execute () {

        this.elevator.setPosition(Constants.ElevatorConstants.LOCK_POSITION);
    }
}
