package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.util.Constants;

public class IntakeNote extends Command {

    private Intake intake;

    public IntakeNote (Intake intake) {

        this.intake = intake;
        this.addRequirements(this.intake);
    }
    
    @Override
    public void initialize () { this.intake.setVelocity(Constants.IntakeConstants.INTAKE_SPEED); }
}
