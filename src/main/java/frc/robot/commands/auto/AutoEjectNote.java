package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.StateMachine;
import frc.robot.commands.indexer.IdleIndexer;
import frc.robot.commands.intake.IdleIntake;
import frc.robot.commands.shooter.IdleShooter;

public class AutoEjectNote extends Command {
    
    private StateMachine intakeStateMachine;
    private StateMachine indexerStateMachine;
    private StateMachine shooterStateMachine;

    public AutoEjectNote (StateMachine intakeStateMachine, StateMachine indexerStateMachine, StateMachine shooterStateMachine) {
        
        this.intakeStateMachine = intakeStateMachine;
        this.indexerStateMachine = indexerStateMachine;
        this.shooterStateMachine = shooterStateMachine;
    }

    @Override
    public void initialize () {

        this.intakeStateMachine.activateState(frc.robot.commands.intake.EjectNote.class);
        this.indexerStateMachine.activateState(frc.robot.commands.indexer.EjectNote.class);
        this.shooterStateMachine.activateState(frc.robot.commands.shooter.EjectNote.class);
    }

    @Override
    public void end (boolean interrupted) {

        this.intakeStateMachine.activateState(IdleIntake.class);
        this.indexerStateMachine.activateState(IdleIndexer.class);
        this.shooterStateMachine.activateState(IdleShooter.class);
    }
}
