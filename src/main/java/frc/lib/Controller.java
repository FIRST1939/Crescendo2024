package frc.lib;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.util.Alerts;

public class Controller extends CommandXboxController implements Subsystem {
    
    private int port;
    private Timer rumbleTimer;

    public Controller (int port) { 
        
        super(port); 
        this.port = port;
        this.rumbleTimer = new Timer();
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

        if (this.rumbleTimer.get() > 0.75) {

            this.getHID().setRumble(RumbleType.kBothRumble, 0.0);
            this.rumbleTimer.reset();
            this.rumbleTimer.stop();
        }
    }

    public void setRumble (RumbleType rumbleType, double value) {

        //this.getHID().setRumble(rumbleType, value);
        this.rumbleTimer.restart();
    }
}
