package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;

public class IdleIndexer extends Command {

    private Indexer indexer;

    public IdleIndexer (Indexer indexer) {

        this.indexer = indexer;
        this.addRequirements(this.indexer);
    }
    
}
