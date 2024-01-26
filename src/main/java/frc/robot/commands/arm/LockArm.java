package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class LockArm extends Command {

    private Arm arm;

    public LockArm (Arm arm) {
        
        this.arm = arm;
        this.addRequirements(this.arm);
    }

}
