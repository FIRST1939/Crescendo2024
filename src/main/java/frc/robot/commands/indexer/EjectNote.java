package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.util.Constants;

public class EjectNote extends Command {
    
    private Indexer indexer;

    public EjectNote (Indexer indexer) {
        
        this.indexer = indexer;
        this.addRequirements(this.indexer);
    }

    @Override
    public void execute () { 
        
        this.indexer.setFrontVelocity(Constants.IndexerConstants.FRONT_EJECT_SPEED); 
        this.indexer.setBackVelocity(Constants.IndexerConstants.BACK_EJECT_SPEED);
    }
}
