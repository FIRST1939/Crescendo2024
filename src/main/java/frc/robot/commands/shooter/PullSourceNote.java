package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.util.Constants;
import frc.robot.util.Sensors;

public class PullSourceNote extends Command {
    
    private Shooter shooter;
    private boolean noteContained = false;

    public PullSourceNote (Shooter shooter) {

        this.shooter = shooter;
        this.addRequirements(this.shooter);
    }

    @Override
    public void execute () { 
        
        if (!Sensors.getIndexerEndBeam()) { this.noteContained = true; }

        this.shooter.setBottomVelocity(Constants.ShooterConstants.PULL_SOURCE_SPEED);
        this.shooter.setTopVelocity(Constants.ShooterConstants.PULL_SOURCE_SPEED);
    }

    @Override
    public boolean isFinished () {

        return this.noteContained && Sensors.getIndexerEndBeam();
    }
}
