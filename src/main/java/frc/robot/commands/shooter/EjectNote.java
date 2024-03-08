package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.util.Constants;

public class EjectNote extends Command {
    
    private Shooter shooter;

    public EjectNote (Shooter shooter) {
        
        this.shooter = shooter;
        this.addRequirements(this.shooter);
    }

    @Override
    public void execute () { 
        
        this.shooter.setTopVelocity(Constants.ShooterConstants.EJECT_SPEED);
        this.shooter.setBottomVelocity(Constants.ShooterConstants.EJECT_SPEED);
    }
}
