package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.util.Constants;

public class ManualLowerSlowElevator extends Command {
    
    private Elevator elevator;

    public ManualLowerSlowElevator (Elevator elevator) {

        this.elevator = elevator;
        this.addRequirements(this.elevator);
    }

    @Override
    public void execute () {

        this.elevator.setInput(Constants.ElevatorConstants.MANUAL_LOWER_SLOW_SPEED);
    }
}
