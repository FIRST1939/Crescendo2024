package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;
import frc.robot.util.Constants.IdleBehavior;
import frc.robot.util.Sensors;

public class Indexer extends SubsystemBase {

    private CANSparkFlex frontRollers;
    private CANSparkMax backRollers;

    private PIDController frontController;
    private PIDController backController;

    public Indexer () {

        this.frontRollers = new CANSparkFlex(Constants.IndexerConstants.FRONT_ROLLERS, MotorType.kBrushless);
        this.backRollers = new CANSparkMax(Constants.IndexerConstants.BACK_ROLLERS, MotorType.kBrushless);

        this.frontRollers.setInverted(Constants.IndexerConstants.FRONT_ROLLERS_INVERTED);
        this.backRollers.setInverted(Constants.IndexerConstants.BACK_ROLLERS_INVERTED);

        this.frontRollers.getEncoder().setPositionConversionFactor(Constants.IndexerConstants.FRONT_ROLLERS_REDUCTION * (Math.PI * Constants.IndexerConstants.FRONT_ROLLERS_DIAMETER));
        this.frontRollers.getEncoder().setVelocityConversionFactor((Constants.IndexerConstants.FRONT_ROLLERS_REDUCTION / 60.0) * (Math.PI * Constants.IndexerConstants.FRONT_ROLLERS_DIAMETER));

        this.backRollers.getEncoder().setPositionConversionFactor(Constants.IndexerConstants.BACK_ROLLERS_REDUCTION * (Math.PI * Constants.IndexerConstants.BACK_ROLLERS_DIAMETER));
        this.backRollers.getEncoder().setVelocityConversionFactor((Constants.IndexerConstants.BACK_ROLLERS_REDUCTION / 60.0) * (Math.PI * Constants.IndexerConstants.BACK_ROLLERS_DIAMETER));

        this.frontController = new PIDController(
            Constants.IndexerConstants.FRONT_SPEED_KP,
            0.0,
            0.0
        );

        this.backController = new PIDController(
            Constants.IndexerConstants.BACK_SPEED_KP,
            0.0,
            0.0
        );
    }

    @Override
    public void periodic () {

        SmartDashboard.putBoolean("Indexer Start Beam", Sensors.getIndexerStartBeam());
        SmartDashboard.putBoolean("Indexer End Beam", Sensors.getIndexerEndBeam());

        SmartDashboard.putNumber("Front Indexer Velocity", this.frontRollers.getEncoder().getVelocity());
        SmartDashboard.putNumber("Back Indexer Velocity", this.backRollers.getEncoder().getVelocity());

        SmartDashboard.putNumber("Front Indexer Position", this.frontRollers.getEncoder().getPosition());
        SmartDashboard.putNumber("Back Indexer Position", this.backRollers.getEncoder().getPosition());
    }

    public void setFrontVelocity (double velocity) {

        if (velocity == 0.0) { 
            
            this.frontRollers.setVoltage(0.0);
            return;
        }

        double feedforward = Constants.IndexerConstants.FRONT_SPEED_KS * Math.signum(velocity) + (Constants.IndexerConstants.FRONT_SPEED_KV * velocity);
        double feedback = this.frontController.calculate(this.frontRollers.getEncoder().getVelocity(), velocity);
        this.frontRollers.setVoltage(feedforward + feedback);
    }

    public void setBackVelocity (double velocity) {

        if (velocity == 0.0) {

            this.backRollers.setVoltage(0.0);
            return;
        }

        double feedforward = Constants.IndexerConstants.BACK_SPEED_KS * Math.signum(velocity) + (Constants.IndexerConstants.BACK_SPEED_KV * velocity);
        double feedback = this.backController.calculate(this.backRollers.getEncoder().getVelocity(), velocity);
        this.backRollers.setVoltage(feedforward + feedback);
    }

    public double getFrontPosition () { return this.frontRollers.getEncoder().getPosition(); }
    public double getBackPosition () { return this.backRollers.getEncoder().getPosition(); }

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
