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
    public void initialize () { this.shooter.setVelocity(Constants.ShooterConstants.SHOOT_SPEED); }
}
