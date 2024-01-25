package frc.lib;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.util.Alerts;

public class Controller extends CommandXboxController implements Subsystem {
    
    private int port;

    public Controller (int port) { 
        
        super(port); 
        this.port = port;
    }

    @Override
    public void periodic () {

        if (!DriverStation.isJoystickConnected(this.port)) { Alerts.driverTwoDisconnected.set(true); } 
        else { Alerts.driverTwoDisconnected.set(false); }
    }
}
