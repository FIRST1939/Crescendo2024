package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.util.Constants;
import frc.robot.util.Sensors;

public class IndexAmpNote extends Command {
    
    private Indexer indexer;
    private Stages stage;

    public IndexAmpNote (Indexer indexer) {

        this.indexer = indexer;
        this.stage = Stages.HANDOFF;
        this.addRequirements(indexer);
    }

    @Override
    public void execute () {
    
        if (this.stage == Stages.HANDOFF) {

            if (!Sensors.getIndexerStartBeam()) {

                this.stage = Stages.LOAD;
                return;
            }

            this.indexer.setFrontVelocity(Constants.IndexerConstants.FRONT_INDEX_SPEED);
        } else if (this.stage == Stages.LOAD) {

            this.indexer.setFrontVelocity(Constants.IndexerConstants.LOAD_SPEED);
        }
    }

    @Override
    public boolean isFinished () {

        return (this.stage == Stages.LOAD && Sensors.getIndexerStartBeam());
    }

    private enum Stages {
        HANDOFF,
        LOAD
    }
}
