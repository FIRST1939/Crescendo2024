package frc.robot.state_machines;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.StateMachine;
import frc.robot.subsystems.Indexer;

public class IndexerStateMachine extends StateMachine {
    
    private Indexer indexer;

    public IndexerStateMachine (ArrayList<Command> states, Indexer indexer) {

        super(states, indexer);
        this.indexer = indexer; 
    }

    protected void generateStateMachineGraph () {

        /**
         * 0: IdleIndexer
         * 1: IndexNote
         * 2: HoldNote
         * 3: FeedNote
         * 4: ReverseNote
         */

        ArrayList<Edge> edges = new ArrayList<>();
        edges.add(new Edge(this.states.get(0), this.states.get(1)));
        edges.add(new Edge(this.states.get(1), this.states.get(2)));
        edges.add(new Edge(this.states.get(1), this.states.get(4)));
        edges.add(new Edge(this.states.get(2), this.states.get(3)));
        edges.add(new Edge(this.states.get(2), this.states.get(4)));
        edges.add(new Edge(this.states.get(3), this.states.get(0)));
        edges.add(new Edge(this.states.get(4), this.states.get(0)));

        this.stateMachineGraph = new Graph(edges);
    }

    protected void switchState () {}
}
