package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.util.Constants;

public class IndexNote extends Command {

    private Indexer indexer;

    public IndexNote (Indexer indexer) {

        this.indexer = indexer;
        this.addRequirements(this.indexer);
    }
    
    @Override
    public void initialize () { this.indexer.setVelocity(Constants.IndexerConstants.INDEX_SPEED); }
}
