package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.util.Constants;
import frc.robot.util.Sensors;

public class IndexSpeakerNote extends Command {
    
    private Indexer indexer;
    private Timer overloadTimer;
    private double loadTarget;
    private Stage stage;

    enum Stage {
        OVERLOAD,
        LOAD
    }

    public IndexSpeakerNote(Indexer indexer) {

        this.indexer = indexer;
        this.overloadTimer = new Timer();
        this.addRequirements(indexer);
    }

    @Override
    public void initialize () {

        this.stage = Stage.OVERLOAD;
        
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

        if (this.stage == Stage.OVERLOAD && this.overloadTimer.get() >= Constants.IndexerConstants.OVERLOAD_TIME) {

            this.stage = Stage.LOAD;
            this.loadTarget = this.indexer.getBackPosition() - Constants.IndexerConstants.LOAD_DISTANCE;
        }

        if (this.stage == Stage.OVERLOAD) {

            double decrease = (this.overloadTimer.get() / Constants.IndexerConstants.OVERLOAD_TIME) * Constants.IndexerConstants.OVERLOAD_DECREASE;

            this.indexer.setFrontVelocity(Constants.IndexerConstants.FRONT_INDEX_SPEED);
            this.indexer.setBackVelocity(Constants.IndexerConstants.BACK_INDEX_SPEED - decrease);
        } else {

            this.indexer.setFrontVelocity(0.0);
            this.indexer.setBackVelocity(Constants.IndexerConstants.LOAD_SPEED);
        }
    }

    @Override
    public boolean isFinished () {

        return (this.stage == Stage.LOAD && this.indexer.getBackPosition() < this.loadTarget);
    }

    @Override
    public void end (boolean interrupted) {

        this.indexer.setFrontVelocity(0.0);
        this.indexer.setBackVelocity(0.0);
    }
}
