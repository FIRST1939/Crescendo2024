package frc.lib;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public abstract class StateMachine {
    
    protected ArrayList<Class<Command>> states;
    private Alert alert;
    private Subsystem[] subsystems;

    protected int currentState;
    protected Command currentCommand;
    protected Graph stateMachineGraph;

    public StateMachine (ArrayList<Class<Command>> states, Alert alert, Subsystem... subsystems) {

        this.states = states;
        this.alert = alert;
        this.subsystems = subsystems;

        this.currentState = 0;
        this.generateStateMachineGraph();
        this.activateState(this.states.get(0));
    }

    protected abstract void generateStateMachineGraph ();
    public abstract void switchState ();

    protected void activateState (Class<Command> state) { 
    
        this.currentCommand.cancel();

        try {

            Command command = state.getConstructor(Subsystem.class).newInstance((Object[]) this.subsystems);
            command.andThen(this::switchState, this.subsystems).schedule();

            this.currentCommand = command;
            this.alert.set(false);
        } catch (Exception exception) {

            this.alert.set(true);
            this.switchState();
        }
    }

    protected class Edge {

        public Class<Command> source;
        public Class<Command> destination;

        public Edge (Class<Command> source, Class<Command> destination) {

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

            public Class<Command> value;
            public Node (Class<Command> value) { this.value = value; }
        }
    }
}
