package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.util.Constants;

public class LockArm extends Command {

    private Arm arm;

    public LockArm (Arm arm) {
        
        this.arm = arm;
        this.addRequirements(this.arm);
    }

    @Override
    public void execute () { 
        
        this.arm.setPosition(Constants.ArmConstants.LOCK_POSITION); 
    }
}
