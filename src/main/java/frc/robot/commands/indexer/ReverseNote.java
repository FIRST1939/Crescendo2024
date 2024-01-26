package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;

public class ReverseNote extends Command {

    private Indexer indexer;

    public ReverseNote (Indexer indexer) {

        this.indexer = indexer;
        this.addRequirements(this.indexer);
    }

}
