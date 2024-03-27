package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.util.Constants;
import frc.robot.util.Sensors;

public class IndexAmpNote extends Command {
    
    private Indexer indexer;
    private Stages stage;
    private int loops;

    public IndexAmpNote (Indexer indexer) {

        this.indexer = indexer;
        this.stage = Stages.INDEX;
        this.addRequirements(indexer);
    }

    @Override
    public void initialize () {

        this.stage = Stages.INDEX;
        this.loops = 0;
    }

    @Override
    public void execute () {
    
        if (loops % 2 == 0 && !Sensors.getIndexerStartBeam()) { this.loops++; }
        else if (loops % 2 == 1 && Sensors.getIndexerStartBeam()) { this.loops++; }

        if (this.stage == Stages.INDEX && this.loops != 0) { this.stage = Stages.POSITION; }

        if (this.stage == Stages.INDEX) {

            this.indexer.setFrontVelocity(Constants.IndexerConstants.FRONT_INDEX_SPEED);
            this.indexer.setBackVelocity(0.0);
        } else if (this.stage == Stages.POSITION) {

            double velocity = -25.0 * Math.pow(-0.925, loops - 1);
            this.indexer.setFrontVelocity(velocity);
            this.indexer.setBackVelocity(0.0);
        }
    }

    public boolean isFinished () {

        return this.loops >= 8;
    }

    private enum Stages {
        INDEX,
        POSITION
    }
}
