package frc.lib;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.util.Alerts;

public class Controller extends CommandXboxController implements Subsystem {
    
    private int port;
    private boolean lastLeftTrigger;
    private boolean lastRightTrigger;

    public Controller (int port) { 
        
        super(port); 
        this.port = port;
    }

    @Override
    public void periodic () {

        if (!DriverStation.isJoystickConnected(this.port)) {

            if (this.port == 0) { Alerts.driverOneDisconnected.set(true);}
            if (this.port == 1) { Alerts.driverTwoDisconnected.set(true); }
        } else {

            if (this.port == 0) { Alerts.driverOneDisconnected.set(false);}
            if (this.port == 1) { Alerts.driverTwoDisconnected.set(false); }
        }
    }

    public Actions getLeftTrigger () {

        boolean current = this.getHID().getLeftTriggerAxis() > 0.5;
        boolean pressed = current && !this.lastLeftTrigger;
        boolean released = !current && this.lastLeftTrigger;
        this.lastLeftTrigger = current;
        
        if (pressed) { return Actions.PRESS; }
        if (released) { return Actions.RELEASE; }
        return null;
    }

    public Actions getRightTrigger () {

        boolean current = this.getHID().getRightTriggerAxis() > 0.5;
        boolean pressed = current && !this.lastRightTrigger;
        boolean released = !current && this.lastRightTrigger;
        this.lastRightTrigger = current;

        if (pressed) { return Actions.PRESS; }
        if (released) { return Actions.RELEASE; }
        return null;
    }

    public enum Actions {
        PRESS,
        RELEASE
    }
}
