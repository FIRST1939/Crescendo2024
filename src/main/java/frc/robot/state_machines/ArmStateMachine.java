package frc.robot.state_machines;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.StateMachine;
import frc.robot.subsystems.Arm;
import frc.robot.util.Alerts;

public class ArmStateMachine extends StateMachine {

    private Arm arm; 

    public ArmStateMachine (ArrayList<Class<Command>> states, ArrayList<ArrayList<Class<?>>> stateParameterTypes, ArrayList<ArrayList<Object>> stateParameters, Arm arm) {

        super(states, stateParameterTypes, stateParameters, Alerts.armStateMachine, arm);
        this.arm = arm;
    }

    protected void generateStateMachineGraph () {

        /**
         * 0: LockArm
         * 1: PivotArm
         */

        ArrayList<Edge> edges = new ArrayList<>();
        edges.add(new Edge(this.states.get(0), this.states.get(1)));
        edges.add(new Edge(this.states.get(1), this.states.get(0)));

        this.stateMachineGraph = new Graph(edges);
    }

    public void switchState () {}
}
