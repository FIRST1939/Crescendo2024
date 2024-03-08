package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;
import frc.robot.util.Constants.IdleBehavior;

public class Indexer extends SubsystemBase {

    private CANSparkMax frontRollers;
    private CANSparkMax backRollers;
    private DigitalInput startBeam;
    private DigitalInput endBeam;

    private Timer feedTimer;

    public Indexer () {

        this.frontRollers = new CANSparkMax(Constants.IndexerConstants.FRONT_ROLLERS, MotorType.kBrushless);
        this.backRollers = new CANSparkMax(Constants.IndexerConstants.BACK_ROLLERS, MotorType.kBrushless);

        this.frontRollers.setInverted(Constants.IndexerConstants.FRONT_ROLLERS_INVERTED);
        this.backRollers.setInverted(Constants.IndexerConstants.BACK_ROLLERS_INVERTED);

        this.frontRollers.getEncoder().setPositionConversionFactor(Constants.IndexerConstants.FRONT_ROLLERS_REDUCTION * (Math.PI * Constants.IndexerConstants.FRONT_ROLLERS_DIAMETER));
        this.frontRollers.getEncoder().setVelocityConversionFactor((Constants.IndexerConstants.FRONT_ROLLERS_REDUCTION / 60.0) * (Math.PI * Constants.IndexerConstants.FRONT_ROLLERS_DIAMETER));

        this.backRollers.getEncoder().setPositionConversionFactor(Constants.IndexerConstants.BACK_ROLLERS_REDUCTION * (Math.PI * Constants.IndexerConstants.BACK_ROLLERS_DIAMETER));
        this.backRollers.getEncoder().setVelocityConversionFactor((Constants.IndexerConstants.BACK_ROLLERS_REDUCTION / 60.0) * (Math.PI * Constants.IndexerConstants.BACK_ROLLERS_DIAMETER));

        this.frontRollers.getPIDController().setP(Constants.IndexerConstants.FRONT_ROLLERS_P);
        this.frontRollers.getPIDController().setI(Constants.IndexerConstants.FRONT_ROLLERS_I);
        this.frontRollers.getPIDController().setD(Constants.IndexerConstants.FRONT_ROLLERS_D);

        this.backRollers.getPIDController().setP(Constants.IndexerConstants.BACK_ROLLERS_P);
        this.backRollers.getPIDController().setI(Constants.IndexerConstants.BACK_ROLLERS_I);
        this.backRollers.getPIDController().setD(Constants.IndexerConstants.BACK_ROLLERS_D);

        this.startBeam = new DigitalInput(Constants.IndexerConstants.START_BEAM);
        this.endBeam = new DigitalInput(Constants.IndexerConstants.END_BEAM);
        this.feedTimer = new Timer();
    }

    @Override
    public void periodic () {

        if (this.endBeam.get() && this.feedTimer.get() == 0.0) { this.feedTimer.start(); }
        else if (!this.endBeam.get()) {

            this.feedTimer.stop();
            this.feedTimer.reset();
        }
    }

    public void setFrontVelocity (double velocity) {

        double frontMax = 6784 * Constants.IndexerConstants.FRONT_ROLLERS_REDUCTION * (Math.PI * Constants.IndexerConstants.FRONT_ROLLERS_DIAMETER) * (1 / 60.0);
        this.frontRollers.set(velocity / frontMax);
    }

    public void setBackVelocity (double velocity) {

        double backMax = 5820 * Constants.IndexerConstants.BACK_ROLLERS_REDUCTION * (Math.PI * Constants.IndexerConstants.BACK_ROLLERS_DIAMETER) * (1 / 60.0);
        this.backRollers.set(velocity / backMax);
    }

    public boolean noteContained () { return !this.startBeam.get(); }
    public boolean noteIndexed () { return !this.endBeam.get(); }
    public boolean noteFed () { return this.endBeam.get() && this.feedTimer.get() > Constants.IndexerConstants.FEED_WAIT; }

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
