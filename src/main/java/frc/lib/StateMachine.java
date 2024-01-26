package frc.lib;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public abstract class StateMachine {
    
    protected ArrayList<Command> states;
    private Subsystem[] subsystems;

    protected Graph stateMachineGraph;
    protected Command currentCommand;

    public StateMachine (ArrayList<Command> states, Subsystem... subsystems) {

        this.states = states;
        this.subsystems = subsystems;

        this.currentCommand = this.states.get(0);

        this.generateStateMachineGraph();
        this.activateState(this.currentCommand);
    }

    protected abstract void generateStateMachineGraph ();
    protected abstract void switchState ();

    protected void activateState (Command state) { state.andThen(() -> this.switchState(), subsystems).schedule(); }

    protected class Edge {

        public Command source;
        public Command destination;

        public Edge (Command source, Command destination) {

            this.source = source;
            this.destination = destination;
        }
    }

    protected class Graph {

        private ArrayList<ArrayList<Node>> adjacencyList = new ArrayList<>();

        public Graph (ArrayList<Edge> edges) {

            for (int i = 1; i <= edges.size(); i++) { this.adjacencyList.add(new ArrayList<Node>()); }
            for (Edge edge : edges) { this.adjacencyList.get(states.indexOf(edge.source)).add(new Node(edge.destination)); }
        }

        protected class Node {

            public Command value;
            public Node (Command value) { this.value = value; }
        }
    }
}
