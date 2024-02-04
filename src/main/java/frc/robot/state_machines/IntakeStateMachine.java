package frc.robot.state_machines;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.StateMachine;
import frc.robot.subsystems.Intake;
import frc.robot.util.Alerts;

public class IntakeStateMachine extends StateMachine {
    
    private Intake intake;

    public IntakeStateMachine (ArrayList<Class<Command>> states, ArrayList<ArrayList<Class<?>>> stateParameterTypes, ArrayList<ArrayList<Object>> stateParameters, Intake intake) {
        
        super(states, stateParameterTypes, stateParameters, Alerts.intakeStateMachine, intake);
        this.intake = intake;
    }

    protected void generateStateMachineGraph () {

        /**
         * 0: IdleIntake
         * 1:IntakeNote
         * 2: OutakeNote
         */

        ArrayList<Edge> edges = new ArrayList<>();
        edges.add(new Edge(this.states.get(0), this.states.get(1)));
        edges.add(new Edge(this.states.get(0), this.states.get(2)));
        edges.add(new Edge(this.states.get(1), this.states.get(0)));
        edges.add(new Edge(this.states.get(1), this.states.get(2)));
        edges.add(new Edge(this.states.get(2), this.states.get(0)));
        edges.add(new Edge(this.states.get(2), this.states.get(1)));

        this.stateMachineGraph = new Graph(edges);
    }

    protected void switchState () {}
}
