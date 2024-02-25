package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IdleIntake extends Command {

    private Intake intake;

    public IdleIntake (Intake intake) {

        this.intake = intake;
        this.addRequirements(this.intake);
    }

    @Override
    public void execute () { this.intake.setVelocity(0.0); }
}
