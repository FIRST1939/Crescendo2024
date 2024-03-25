package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.util.Constants;

public class DropNote extends Command {
    
    private Indexer indexer;
    private Timer dropTimer;

    public DropNote(Indexer indexer) {

        this.indexer = indexer;
        this.dropTimer = new Timer();
        this.addRequirements(indexer);
    }

    @Override
    public void initialize() {

        this.dropTimer.stop();
        this.dropTimer.reset();
    }

    @Override
    public void execute () {

        this.indexer.setFrontVelocity(Constants.IndexerConstants.DROP_SPEED);
    }

    @Override
    public boolean isFinished () {

        return (this.dropTimer.get() >= Constants.IndexerConstants.DROP_WAIT);
    }

    @Override
    public void end (boolean interrupted) {

        this.indexer.setFrontVelocity(0.0);
    }
}
