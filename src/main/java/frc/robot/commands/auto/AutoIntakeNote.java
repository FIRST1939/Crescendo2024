package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.StateMachine;
import frc.robot.commands.intake.IntakeNote;

public class AutoIntakeNote extends InstantCommand {
    
    private StateMachine intakeStateMachine;

    public AutoIntakeNote (StateMachine intakeStateMachine) {
        
        this.intakeStateMachine = intakeStateMachine;
    }
    
    @Override
    public void initialize () {

        this.intakeStateMachine.activateState(IntakeNote.class);
    }
}
