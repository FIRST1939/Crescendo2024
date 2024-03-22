package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.util.Constants;
import frc.robot.util.Sensors;

public class PivotArm extends Command {
    
    private Arm arm; 
    private Timer feedTimer;

    public PivotArm (Arm arm) {

        this.arm = arm;
        this.feedTimer = new Timer();
        this.addRequirements(this.arm);
    }

    @Override
    public void initialize () { 
        
        this.arm.manualPivotAdjustment = 0.0; 
    }

    @Override
    public void execute () { 
        
        if (Sensors.getIndexerEndBeam() && this.feedTimer.get() == 0.0) { this.feedTimer.start(); }

        double armPosition = Constants.ArmConstants.PIVOT_POSITION + this.arm.manualPivotAdjustment;
        this.arm.setPosition(armPosition); 
    }

    @Override
    public boolean isFinished () { 
        
        return (this.feedTimer.get() >= Constants.IndexerConstants.FEED_WAIT); 
    }
}
