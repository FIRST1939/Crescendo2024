package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShootNote extends Command {

    private Shooter shooter;

    public ShootNote (Shooter shooter) {

        this.shooter = shooter;
        this.addRequirements(this.shooter);
    }

    @Override
    public void execute () { 
        
        this.shooter.setTopVelocity(this.shooter.getSpeed()); 
        this.shooter.setBottomVelocity(this.shooter.getSpeed());
    }
}
