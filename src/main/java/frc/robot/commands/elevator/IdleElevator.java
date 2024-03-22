package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class IdleElevator extends Command {
    
    private Elevator elevator;

    public IdleElevator (Elevator elevator) {

        this.elevator = elevator;
        this.addRequirements(this.elevator);
    }

    @Override
    public void initialize () {

        this.elevator.setInput(0.0);
    }
}
