package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;

public class HoldNote extends Command {

    private Indexer indexer;

    public HoldNote (Indexer indexer) {

        this.indexer = indexer;
        this.addRequirements(this.indexer);
    }
    
    @Override
    public void execute () { 
        
        this.indexer.setFrontVelocity(0.0); 
        this.indexer.setBackVelocity(0.0);
    }
}
