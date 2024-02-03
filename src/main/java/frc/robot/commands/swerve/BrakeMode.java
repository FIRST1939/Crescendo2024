package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Swerve;

public class BrakeMode extends InstantCommand {
    
    private Swerve swerve;
    private boolean brake;

    public BrakeMode (Swerve swerve, boolean brake) {

        this.swerve = swerve;
        this.brake = brake;
    }

    @Override
    public void initialize () { 
        
        if (!DriverStation.isEnabled()) { 
            
            this.swerve.setBrakeMode(brake); 
        }
    }

    @Override
    public boolean isFinished () { return true; }
}
