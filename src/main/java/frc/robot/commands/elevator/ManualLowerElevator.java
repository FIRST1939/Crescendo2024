package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.util.Constants;

public class ManualLowerElevator extends Command {
    
    private Elevator elevator;

    public ManualLowerElevator (Elevator elevator) {

        this.elevator = elevator;
        this.addRequirements(this.elevator);
    }

    @Override
    public void execute () {

        this.elevator.setVelocity(Constants.ElevatorConstants.LOWER_SPEED);
    }
}
