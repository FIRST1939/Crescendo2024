package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.StateMachine;
import frc.robot.commands.arm.LockArm;
import frc.robot.commands.arm.PivotArm;
import frc.robot.commands.indexer.IndexSpeakerNote;
import frc.robot.commands.shooter.IdleShooter;
import frc.robot.commands.shooter.ShootNote;

public class AutoScoreNote extends Command {
    
    private StateMachine indexerStateMachine;
    private StateMachine armStateMachine;
    private StateMachine shooterStateMachine;

    public AutoScoreNote (StateMachine indexerStateMachine, StateMachine armStateMachine, StateMachine shooterStateMachine) {
        
        this.indexerStateMachine = indexerStateMachine;
        this.armStateMachine = armStateMachine;
        this.shooterStateMachine = shooterStateMachine;
    }

    @Override
    public void initialize () {

        this.armStateMachine.activateState(PivotArm.class);
        this.shooterStateMachine.activateState(ShootNote.class);
    }

    @Override
    public boolean isFinished () {

        return (this.indexerStateMachine.getCurrentState() == IndexSpeakerNote.class);
    }
}
