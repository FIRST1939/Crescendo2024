package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.util.Constants;

public class PivotArm extends Command {
    
    private Arm arm; 

    public PivotArm (Arm arm) {

        this.arm = arm;
        this.addRequirements(this.arm);
    }

    @Override
    public void initialize () { this.arm.setPosition(Constants.ArmConstants.PIVOT_POSITION); }
}
