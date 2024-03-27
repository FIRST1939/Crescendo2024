package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.util.Constants;
import frc.robot.util.Sensors;

public class IndexAmpNote extends Command {
    
    private Indexer indexer;
    private Stages stage;

    private int loops;
    private double targetPosition;

    public IndexAmpNote (Indexer indexer) {

        this.indexer = indexer;
        this.stage = Stages.INDEX;
        this.addRequirements(indexer);
    }

    @Override
    public void initialize () {

        this.stage = Stages.INDEX;
        this.loops = 0;
    }

    @Override
    public void execute () {
    
        if (loops % 2 == 0 && !Sensors.getIndexerStartBeam()) { this.loops++; }
        else if (loops % 2 == 1 && Sensors.getIndexerStartBeam()) { this.loops++; }

        if (this.stage == Stages.INDEX && this.loops != 0) { this.stage = Stages.POSITION; }

        if (this.stage == Stages.POSITION && this.loops >= 8) { 
            
            this.stage = Stages.HOLD; 
            this.targetPosition = this.indexer.getFrontPosition() + 2.0;
        }

        if (this.stage == Stages.INDEX) {

            this.indexer.setFrontVelocity(Constants.IndexerConstants.FRONT_INDEX_SPEED);
            this.indexer.setBackVelocity(0.0);
        } else if (this.stage == Stages.POSITION) {

            double velocity = -25.0 * Math.pow(-0.925, loops - 1);
            this.indexer.setFrontVelocity(velocity);
            this.indexer.setBackVelocity(0.0);
        } else if (this.stage == Stages.HOLD) {

            double velocity = 20.0 * (this.targetPosition - this.indexer.getFrontPosition());
            this.indexer.setFrontVelocity(velocity);
            this.indexer.setBackVelocity(0.0);
        }

        SmartDashboard.putNumber("Error", (this.targetPosition - this.indexer.getFrontPosition()));
    }
    
    @Override
    public boolean isFinished () {

        return false;
    }

    private enum Stages {
        INDEX,
        POSITION,
        HOLD
    }
}
