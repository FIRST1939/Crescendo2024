package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.Controller;

public class RobotContainer {
    
    private CommandXboxController driverTwo;

    public RobotContainer () {

        this.driverTwo = new Controller(1);
        this.configureCommands();
    }

    private void configureCommands () {}
}
