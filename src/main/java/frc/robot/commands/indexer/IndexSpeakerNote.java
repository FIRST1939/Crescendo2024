package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.util.Constants;
import frc.robot.util.Sensors;

public class IndexSpeakerNote extends Command {
    
    private Indexer indexer;
    private Timer overloadTimer;

    public IndexSpeakerNote(Indexer indexer) {

        this.indexer = indexer;
        this.overloadTimer = new Timer();

        this.addRequirements(indexer);
    }

    @Override
    public void initialize () {

        this.overloadTimer.stop();
        this.overloadTimer.reset();
    }

    @Override
    public void execute () {
    
        if (!Sensors.getIndexerEndBeam()) {

            this.overloadTimer.start();
        } else {

            this.overloadTimer.stop();
            this.overloadTimer.reset();
        }

        this.indexer.setFrontVelocity(Constants.IndexerConstants.FRONT_INDEX_SPEED);
        this.indexer.setBackVelocity(Constants.IndexerConstants.BACK_INDEX_SPEED);
    }

    @Override
    public boolean isFinished () {

        return (this.overloadTimer.get() >= Constants.IndexerConstants.OVERLOAD_TIME);
    }
}
