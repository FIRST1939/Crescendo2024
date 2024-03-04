package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.StateMachine;
import frc.robot.commands.arm.PivotArm;
import frc.robot.commands.shooter.ShootNote;

public class AutoScoreNote extends InstantCommand {
    
    private StateMachine armStateMachine;
    private StateMachine shooterStateMachine;

    public AutoScoreNote (StateMachine armStateMachine, StateMachine shooterStateMachine) {
        
        this.armStateMachine = armStateMachine;
        this.shooterStateMachine = shooterStateMachine;
    }

    @Override
    public void initialize () {

        this.armStateMachine.activateState(PivotArm.class);
        this.shooterStateMachine.activateState(ShootNote.class);
    }
}
