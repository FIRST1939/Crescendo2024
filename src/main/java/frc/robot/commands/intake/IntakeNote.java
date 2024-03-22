package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.util.Constants;
import frc.robot.util.Sensors;

public class IntakeNote extends Command {

    private Intake intake;

    public IntakeNote (Intake intake) {

        this.intake = intake;
        this.addRequirements(this.intake);
    }
    
    @Override
    public void execute () { 
        
        this.intake.setVelocity(Constants.IntakeConstants.INTAKE_SPEED); 
    }

    @Override
    public boolean isFinished () {

        return (!Sensors.getIndexerStartBeam());
    }
}
