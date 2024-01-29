package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class OutakeNote extends Command {

    private Intake intake;

    public OutakeNote (Intake intake) {

        this.intake = intake;
        this.addRequirements(this.intake);
    }
    
}
