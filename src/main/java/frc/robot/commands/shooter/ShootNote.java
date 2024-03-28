package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.util.Constants;

public class ShootNote extends Command {

    private Shooter shooter;

    public ShootNote (Shooter shooter) {

        this.shooter = shooter;
        this.addRequirements(this.shooter);
    }

    @Override
    public void execute () { 
        
        this.shooter.setTopVelocity(Constants.ShooterConstants.TOP_SHOOT_SPEED); 
        this.shooter.setBottomVelocity(Constants.ShooterConstants.BOTTOM_SHOOT_SPEED);
    }
}
