package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;
import frc.robot.util.Constants.IdleBehavior;
import frc.robot.util.MotorUtil;

public class Indexer extends SubsystemBase {

    private CANSparkFlex frontRollers;
    private CANSparkMax backRollers;
    
    private DigitalInput startBeam;
    private DigitalInput endBeam;

    private Timer loadTimer;
    private Timer feedTimer;

    public Indexer () {

        this.frontRollers = new CANSparkFlex(Constants.IndexerConstants.FRONT_ROLLERS, MotorType.kBrushless);
        this.backRollers = new CANSparkMax(Constants.IndexerConstants.BACK_ROLLERS, MotorType.kBrushless);

        this.frontRollers.setInverted(Constants.IndexerConstants.FRONT_ROLLERS_INVERTED);
        this.backRollers.setInverted(Constants.IndexerConstants.BACK_ROLLERS_INVERTED);

        this.frontRollers.getEncoder().setPositionConversionFactor(Constants.IndexerConstants.FRONT_ROLLERS_REDUCTION * (Math.PI * Constants.IndexerConstants.FRONT_ROLLERS_DIAMETER));
        this.frontRollers.getEncoder().setVelocityConversionFactor((Constants.IndexerConstants.FRONT_ROLLERS_REDUCTION / 60.0) * (Math.PI * Constants.IndexerConstants.FRONT_ROLLERS_DIAMETER));

        this.backRollers.getEncoder().setPositionConversionFactor(Constants.IndexerConstants.BACK_ROLLERS_REDUCTION * (Math.PI * Constants.IndexerConstants.BACK_ROLLERS_DIAMETER));
        this.backRollers.getEncoder().setVelocityConversionFactor((Constants.IndexerConstants.BACK_ROLLERS_REDUCTION / 60.0) * (Math.PI * Constants.IndexerConstants.BACK_ROLLERS_DIAMETER));

        this.startBeam = new DigitalInput(Constants.IndexerConstants.START_BEAM);
        this.endBeam = new DigitalInput(Constants.IndexerConstants.END_BEAM);

        this.loadTimer = new Timer();
        this.feedTimer = new Timer();
    }

    @Override
    public void periodic () {

        SmartDashboard.putBoolean("End Beam", this.endBeam.get());

        if (!this.endBeam.get() && this.loadTimer.get() == 0.0) { this.loadTimer.start(); }
        else if (this.endBeam.get()) {

            this.loadTimer.stop();
            this.loadTimer.reset();
        }

        if (this.endBeam.get() && this.feedTimer.get() == 0.0) { this.feedTimer.start(); }
        else if (!this.endBeam.get()) {

            this.feedTimer.stop();
            this.feedTimer.reset();
        }
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

    public boolean noteContained () { return !(this.startBeam.get() && this.endBeam.get()); }
    public boolean noteIndexed () { return !this.endBeam.get() && this.loadTimer.get() > Constants.IndexerConstants.LOAD_TIME; }
    public boolean noteFed () { return this.endBeam.get() && this.feedTimer.get() > Constants.IndexerConstants.FEED_WAIT; }

    public boolean getBeamBreak () { return this.endBeam.get(); }

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
