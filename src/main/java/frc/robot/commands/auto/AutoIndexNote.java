package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.StateMachine;
import frc.robot.commands.indexer.IndexNote;

public class AutoIndexNote extends InstantCommand {
    
    private StateMachine indexStateMachine;

    public AutoIndexNote (StateMachine indexStateMachine) {
        
        this.indexStateMachine = indexStateMachine;
    }

    @Override
    public void initialize () {

        this.indexStateMachine.activateState(IndexNote.class);
    }
}
