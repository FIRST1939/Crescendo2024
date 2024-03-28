package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;

public class HoldAmpNote extends Command {
    
    private Indexer indexer;
    private double targetPosition;

    public HoldAmpNote (Indexer indexer) {

        this.indexer = indexer;
        this.addRequirements(indexer);
    }

    @Override
    public void initialize () {

        this.targetPosition = this.indexer.getFrontPosition() + 2.0;
    }

    @Override
    public void execute () {

        double velocity = 20.0 * (this.targetPosition - this.indexer.getFrontPosition());
        this.indexer.setFrontVelocity(velocity);
        this.indexer.setBackVelocity(0.0);
    }
}
