package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;

public class HoldAmpNote extends Command {
    
    private Indexer indexer;

    public HoldAmpNote (Indexer indexer) {

        this.indexer = indexer;
        this.addRequirements(indexer);
    }

    @Override
    public void initialize () {

        this.indexer.setFrontVelocity(0.0);
        this.indexer.setBackVelocity(0.0);
    }
}
