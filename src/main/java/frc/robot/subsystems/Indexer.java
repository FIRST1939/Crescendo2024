package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;
import frc.robot.util.Constants.IdleBehavior;
import frc.robot.util.MotorUtil;

public class Indexer extends SubsystemBase {

    private CANSparkFlex frontRollers;
    private CANSparkMax backRollers;

    public Indexer () {

        this.frontRollers = new CANSparkFlex(Constants.IndexerConstants.FRONT_ROLLERS, MotorType.kBrushless);
        this.backRollers = new CANSparkMax(Constants.IndexerConstants.BACK_ROLLERS, MotorType.kBrushless);

        this.frontRollers.setInverted(Constants.IndexerConstants.FRONT_ROLLERS_INVERTED);
        this.backRollers.setInverted(Constants.IndexerConstants.BACK_ROLLERS_INVERTED);

        this.frontRollers.getEncoder().setPositionConversionFactor(Constants.IndexerConstants.FRONT_ROLLERS_REDUCTION * (Math.PI * Constants.IndexerConstants.FRONT_ROLLERS_DIAMETER));
        this.frontRollers.getEncoder().setVelocityConversionFactor((Constants.IndexerConstants.FRONT_ROLLERS_REDUCTION / 60.0) * (Math.PI * Constants.IndexerConstants.FRONT_ROLLERS_DIAMETER));

        this.backRollers.getEncoder().setPositionConversionFactor(Constants.IndexerConstants.BACK_ROLLERS_REDUCTION * (Math.PI * Constants.IndexerConstants.BACK_ROLLERS_DIAMETER));
        this.backRollers.getEncoder().setVelocityConversionFactor((Constants.IndexerConstants.BACK_ROLLERS_REDUCTION / 60.0) * (Math.PI * Constants.IndexerConstants.BACK_ROLLERS_DIAMETER));
    }

    public void setFrontVelocity (double velocity) {

        double maximumVelocity = MotorUtil.getMaxVelocity(
            frc.robot.util.MotorUtil.MotorType.VORTEX,
            Constants.IndexerConstants.FRONT_ROLLERS_DIAMETER,
            Constants.IndexerConstants.FRONT_ROLLERS_REDUCTION
        );
        
        this.frontRollers.set(velocity / maximumVelocity);
    }

    public void setBackVelocity (double velocity) {

        double maximumVelocity = MotorUtil.getMaxVelocity(
            frc.robot.util.MotorUtil.MotorType.NEO,
            Constants.IndexerConstants.BACK_ROLLERS_DIAMETER,
            Constants.IndexerConstants.BACK_ROLLERS_REDUCTION
        );

        this.backRollers.set(velocity / maximumVelocity);
    }

    public void setIdleBehavior (IdleBehavior idleBehavior) {

        if (idleBehavior == IdleBehavior.COAST) {

            this.frontRollers.setIdleMode(IdleMode.kCoast);
            this.backRollers.setIdleMode(IdleMode.kCoast);
        } else if (idleBehavior == IdleBehavior.BRAKE) {

            this.frontRollers.setIdleMode(IdleMode.kBrake);
            this.backRollers.setIdleMode(IdleMode.kBrake);
        }
    }
}
