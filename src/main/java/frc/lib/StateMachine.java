package frc.lib;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class StateMachine {
    
    private Alert alert;
    private Subsystem[] subsystems;

    private Command currentCommand;
    private Class<? extends Command> currentState;

    public StateMachine (Alert alert, Subsystem... subsystems) {

        this.alert = alert;
        this.subsystems = subsystems;
    }

    public Class<? extends Command> getCurrentState () { return this.currentState; }
    public boolean currentCommandFinished () { return this.currentCommand.isFinished(); }

    public void activateState (Class<? extends Command> state) { 
    
        if (this.currentCommand != null) { this.currentCommand.cancel(); }
        this.currentState = state;

        try {

            Command command = (Command) state.getConstructors()[0].newInstance((Object[]) this.subsystems);
            this.currentCommand = command;

            this.currentCommand.schedule();
            this.alert.set(false);
        } catch (Exception exception) {

            this.alert.set(true);
            exception.printStackTrace();
        }
    }
}
