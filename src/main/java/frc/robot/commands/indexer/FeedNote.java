package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.util.Constants;
import frc.robot.util.Sensors;

public class FeedNote extends Command {

    private Indexer indexer;
    private Timer feedTimer;

    public FeedNote (Indexer indexer) {

        this.indexer = indexer;
        this.feedTimer = new Timer();
        this.addRequirements(this.indexer);
    }

    @Override
    public void initialize () { 
    
        this.feedTimer.stop();
        this.feedTimer.reset(); 
    }
    
    @Override
    public void execute () { 
        
        if (Sensors.getIndexerEndBeam()) { this.feedTimer.start(); }
        this.indexer.setBackVelocity(Constants.IndexerConstants.FEED_SPEED); 
    }

    @Override
    public boolean isFinished () {

        return (this.feedTimer.get() >= Constants.IndexerConstants.FEED_WAIT);
    }
}
