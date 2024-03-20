package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.util.Constants.IdleBehavior;

public class IdleShooter extends Command {

    private Shooter shooter; 
    

    public IdleShooter (Shooter shooter) {

        this.shooter = shooter;
        this.addRequirements(this.shooter);
    }

    @Override
    public void initialize () {

        this.shooter.setIdleBehavior(IdleBehavior.BRAKE);
    }

    @Override
    public void execute () { 
        
        this.shooter.setTopVelocity(0.0); 
        this.shooter.setBottomVelocity(0.0);
    }

    @Override
    public void end (boolean interrupted) {

        this.shooter.setIdleBehavior(IdleBehavior.COAST);
    }
}
