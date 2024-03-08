package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.util.Constants;

public class EjectNote extends Command {
    
    private Intake intake;

    public EjectNote (Intake intake) {
        
        this.intake = intake;
        this.addRequirements(this.intake);
    }

    @Override
    public void execute () { 
        
        this.intake.setVelocity(Constants.IntakeConstants.EJECT_SPEED);
    }
}
