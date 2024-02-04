package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;

public class Shooter extends SubsystemBase{

    private CANSparkFlex topRollers;
    private CANSparkFlex bottomRollers;

    public Shooter () {

        this.topRollers = new CANSparkFlex(Constants.ShooterConstants.TOP_ROLLERS, MotorType.kBrushless);
        this.bottomRollers = new CANSparkFlex(Constants.ShooterConstants.BOTTOM_ROLLERS, MotorType.kBrushless);

        this.topRollers.setInverted(Constants.ShooterConstants.TOP_ROLLERS_INVERTED);
        this.bottomRollers.setInverted(Constants.ShooterConstants.BOTTOM_ROLLERS_INVERTED);
    }
}
