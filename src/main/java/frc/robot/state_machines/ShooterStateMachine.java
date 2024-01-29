package frc.robot.state_machines;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.StateMachine;
import frc.robot.subsystems.Shooter;

public class ShooterStateMachine extends StateMachine {

    private Shooter shooter;

    public ShooterStateMachine (ArrayList<Command> states, Shooter shooter) {

        super(states, shooter);
        this.shooter = shooter; 
    }

    protected void generateStateMachineGraph () {

        /**
         * 0: IdleShooter
         * 1: ShootNote
         */

        ArrayList<Edge> edges = new ArrayList<>();
        edges.add(new Edge(this.states.get(0), this.states.get(1)));
        edges.add(new Edge(this.states.get(1), this.states.get(0)));

        this.stateMachineGraph = new Graph(edges);
    }

    protected void switchState () {}
}

