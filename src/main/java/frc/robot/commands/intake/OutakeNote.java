package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.util.Constants;

public class OutakeNote extends Command {

    private Intake intake;

    public OutakeNote (Intake intake) {

        this.intake = intake;
        this.addRequirements(this.intake);
    }
    
    @Override
    public void execute () { this.intake.setVelocity(Constants.IntakeConstants.OUTAKE_SPEED); }
}
