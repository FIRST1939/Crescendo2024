package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.util.Constants;

public class DropNote extends Command {
    
    private Indexer indexer;
    private Stage stage;
    private Timer retractTimer;
    private Timer dropTimer;

    enum Stage {
        RETRACT,
        SCORE
    }
    
    public DropNote(Indexer indexer) {

        this.indexer = indexer;
        this.retractTimer = new Timer();
        this.dropTimer = new Timer();

        this.addRequirements(indexer);
    }

    @Override
    public void initialize() {

        this.stage = Stage.RETRACT;
        this.retractTimer.restart();
        this.dropTimer.stop();
        this.dropTimer.reset();
    }

    @Override
    public void execute () {

        if (this.stage == Stage.RETRACT && this.retractTimer.get() >= Constants.IndexerConstants.RETRACT_WAIT) {

            this.stage = Stage.SCORE;
            this.dropTimer.start();
        }

        if (this.stage == Stage.RETRACT) {

            this.indexer.setFrontVelocity(Constants.IndexerConstants.RETRACT_SPEED);
        } else {

            this.indexer.setFrontVelocity(Constants.IndexerConstants.DROP_SPEED);
        }
    }

    @Override
    public boolean isFinished () {

        return (this.stage == Stage.SCORE && this.dropTimer.get() >= Constants.IndexerConstants.DROP_WAIT);
    }

    @Override
    public void end (boolean interrupted) {

        this.indexer.setFrontVelocity(0.0);
    }
}
