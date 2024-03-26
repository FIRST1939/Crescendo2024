package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.util.Constants;
import frc.robot.util.Sensors;

public class ShootNote extends Command {

    private Shooter shooter;
    private Timer feedTimer;

    public ShootNote (Shooter shooter) {

        this.shooter = shooter;
        this.feedTimer = new Timer();
        this.addRequirements(this.shooter);
    }

    @Override
    public void initialize () { 
        
        this.feedTimer.stop();
        this.feedTimer.reset(); 
    }

    @Override
    public void execute () { 
        
        if (Sensors.getIndexerEndBeam() && this.feedTimer.get() == 0.0) { this.feedTimer.start(); }
        
        this.shooter.setTopVelocity(Constants.ShooterConstants.TOP_SHOOT_SPEED); 
        this.shooter.setBottomVelocity(Constants.ShooterConstants.BOTTOM_SHOOT_SPEED);
    }

    @Override
    public boolean isFinished () {

        return (this.feedTimer.get() >= Constants.IndexerConstants.FEED_WAIT);
    }
}
