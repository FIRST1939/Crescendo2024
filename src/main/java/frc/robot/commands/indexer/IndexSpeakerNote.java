package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.util.Constants;
import frc.robot.util.Sensors;

public class IndexSpeakerNote extends Command {
    
    private Indexer indexer;
    private Stages stage;

    private Timer overloadTimer;
    private Timer loadTimer;

    public IndexSpeakerNote(Indexer indexer) {

        this.indexer = indexer;
        this.stage = Stages.HANDOFF;

        this.overloadTimer = new Timer();
        this.loadTimer = new Timer();

        this.addRequirements(indexer);
    }

    @Override
    public void execute () {
    
        if (!Sensors.getIndexerEndBeam() && this.overloadTimer.get() == 0.0) {

            this.overloadTimer.start();
        } else if (Sensors.getIndexerEndBeam()) {

            this.overloadTimer.stop();
            this.overloadTimer.reset();
        }

        if (this.stage == Stages.HANDOFF) {

            if (this.overloadTimer.get() >= Constants.IndexerConstants.OVERLOAD_TIME) {

                this.stage = Stages.LOAD;
                this.loadTimer.start();
                return;
            }

            this.indexer.setFrontVelocity(Constants.IndexerConstants.FRONT_INDEX_SPEED);
            this.indexer.setBackVelocity(Constants.IndexerConstants.BACK_INDEX_SPEED);
        } else if (this.stage == Stages.LOAD) {

            this.indexer.setBackVelocity(Constants.IndexerConstants.LOAD_SPEED);
        }
    }

    @Override
    public boolean isFinished () {

        return (this.loadTimer.get() >= Constants.IndexerConstants.LOAD_TIME);
    }

    private enum Stages {
        HANDOFF,
        LOAD
    }
}
