package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.util.Constants;

public class LoadNote extends Command {
    
    private Indexer indexer;
    private Timer timer;

    public LoadNote (Indexer indexer) {

        this.indexer = indexer;
        this.timer = new Timer();

        this.addRequirements(this.indexer);
    }

    @Override
    public void initialize () {

        this.timer.restart();
    }

    @Override
    public void execute () { 
        
        if (this.timer.get() > Constants.IndexerConstants.LOAD_WAIT) {

            this.indexer.setBackVelocity(Constants.IndexerConstants.LOAD_SPEED);
        } else {

            this.indexer.setFrontVelocity(Constants.IndexerConstants.FRONT_INDEX_SPEED);
            this.indexer.setBackVelocity(Constants.IndexerConstants.BACK_INDEX_SPEED);
        }
    }
}
