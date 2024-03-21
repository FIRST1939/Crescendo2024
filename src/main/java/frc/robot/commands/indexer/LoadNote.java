package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.util.Constants;

public class LoadNote extends Command {
    
    private Indexer indexer;

    public LoadNote (Indexer indexer) {

        this.indexer = indexer;
        this.addRequirements(this.indexer);
    }

    @Override
    public void initialize () {

        this.indexer.restartLoadTimer();
    }

    @Override
    public void execute () {

        this.indexer.setFrontVelocity(Constants.IndexerConstants.LOAD_SPEED);
        this.indexer.setBackVelocity(Constants.IndexerConstants.LOAD_SPEED);
    }
}
