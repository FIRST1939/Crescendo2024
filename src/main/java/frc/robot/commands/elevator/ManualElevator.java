package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ManualElevator extends Command {
    
    private Elevator elevator;

    public ManualElevator (Elevator elevator) {

        this.elevator = elevator;
        this.addRequirements(this.elevator);
    }

    @Override
    public void execute () {

        this.elevator.setInput(this.elevator.getSpeed());
    }
}
