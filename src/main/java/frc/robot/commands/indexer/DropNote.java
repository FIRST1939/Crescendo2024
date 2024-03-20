package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.util.Constants;

public class DropNote extends Command {
    
    private Indexer indexer;

    public DropNote(Indexer indexer) {

        this.indexer = indexer;
        this.addRequirements(indexer);
    }

    @Override
    public void execute () {

        this.indexer.setFrontVelocity(Constants.IndexerConstants.DROP_SPEED);
    }
}
