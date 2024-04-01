package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;
import frc.robot.util.Constants.IdleBehavior;

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

        double feedforward = 0.19 + (0.021 * velocity);
        double feedback = 0.013 * (velocity - this.frontRollers.getEncoder().getVelocity());
        this.frontRollers.setVoltage(feedforward + feedback);
    }

    public void setBackVelocity (double velocity) {

        double feedforward = 0.13 + (0.019 * velocity);
        double feedback = 0.013 * (velocity - this.backRollers.getEncoder().getVelocity());
        this.backRollers.setVoltage(feedforward + feedback);
    }

    public double getFrontPosition () { return this.frontRollers.getEncoder().getPosition(); }

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
