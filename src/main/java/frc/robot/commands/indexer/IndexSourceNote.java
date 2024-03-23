package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.util.Constants;
import frc.robot.util.Sensors;

public class IndexSourceNote extends Command {
    
    private Indexer indexer;
    private boolean noteContained = false;

    public IndexSourceNote(Indexer indexer) {

        this.indexer = indexer;
        this.addRequirements(indexer);
    }

    @Override
    public void execute () {

        if (!Sensors.getIndexerEndBeam()) { this.noteContained = true; }
        this.indexer.setBackVelocity(Constants.IndexerConstants.SOURCE_SPEED);
    }

    @Override
    public boolean isFinished () {

        return this.noteContained && Sensors.getIndexerEndBeam();
    }
}
