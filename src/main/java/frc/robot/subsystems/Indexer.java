package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.util.Constants;
import frc.robot.util.Constants.IdleBehavior;
import frc.robot.util.MotorUtil;

public class Indexer extends SubsystemBase {

    private CANSparkMax frontRollers;
    private CANSparkFlex backRollers;
    private DigitalInput startBeam;
    private DigitalInput endBeam;

    private Timer feedTimer;

    public Indexer () {

        this.frontRollers = new CANSparkMax(Constants.IndexerConstants.FRONT_ROLLERS, MotorType.kBrushless);
        this.backRollers = new CANSparkFlex(Constants.IndexerConstants.BACK_ROLLERS, MotorType.kBrushless);

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

        double maximumVelocity = MotorUtil.getMaxVelocity(
            frc.robot.util.MotorUtil.MotorType.NEO,
            Constants.IndexerConstants.FRONT_ROLLERS_DIAMETER,
            Constants.IndexerConstants.FRONT_ROLLERS_REDUCTION
        );
        
        this.frontRollers.set(velocity / maximumVelocity);
    }

    public void setBackVelocity (double velocity) {

        
        double maximumVelocity = MotorUtil.getMaxVelocity(
            frc.robot.util.MotorUtil.MotorType.VORTEX,
            Constants.IndexerConstants.BACK_ROLLERS_DIAMETER,
            Constants.IndexerConstants.BACK_ROLLERS_REDUCTION
        );

        this.backRollers.set(velocity / maximumVelocity);
    }

    public boolean noteContained () { return !(this.startBeam.get() && this.endBeam.get()); }
    public boolean noteIndexed () { return !this.endBeam.get(); }
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

    public Command getFrontQuasistaticRoutine (Direction direction) { return this.getFrontSysIdRoutine().quasistatic(direction); }
    public Command getFrontDynamicRoutine (Direction direction) { return this.getFrontSysIdRoutine().dynamic(direction); }
    
    public Command getBackQuasistaticRoutine (Direction direction) { return this.getBackSysIdRoutine().quasistatic(direction); }
    public Command getBackDynamicRoutine (Direction direction) { return this.getBackSysIdRoutine().dynamic(direction); }

    private SysIdRoutine getFrontSysIdRoutine () {

        return new SysIdRoutine(
            Constants.IndexerConstants.FRONT_SYSID_ROUTINE_CONFIG,
            new Mechanism(
                this::setFrontRollersVoltage,
                sysIdRoutineLog -> {

                    sysIdRoutineLog.motor("indexer-front-rollers")
                        .linearPosition(Units.Inches.of(this.frontRollers.getEncoder().getPosition()))
                        .linearVelocity(Units.InchesPerSecond.of(this.frontRollers.getEncoder().getVelocity()))
                        .voltage(Units.Volts.of(this.frontRollers.getBusVoltage() * this.frontRollers.getAppliedOutput()));
                },
                this
            )
        );
    }

    private SysIdRoutine getBackSysIdRoutine () {

        return new SysIdRoutine(
            Constants.IndexerConstants.BACK_SYSID_ROUTINE_CONFIG,
            new Mechanism(
                this::setBackRollersVoltage,
                sysIdRoutineLog -> {

                    sysIdRoutineLog.motor("indexer-back-rollers")
                        .linearPosition(Units.Inches.of(this.backRollers.getEncoder().getPosition()))
                        .linearVelocity(Units.InchesPerSecond.of(this.backRollers.getEncoder().getVelocity()))
                        .voltage(Units.Volts.of(this.backRollers.getBusVoltage() * this.backRollers.getAppliedOutput()));
                },
                this
            )
        );
    }

    private void setFrontRollersVoltage (Measure<Voltage> voltage) { this.frontRollers.getPIDController().setReference(voltage.magnitude(), ControlType.kVoltage); }
    private void setBackRollersVoltage (Measure<Voltage> voltage) { this.backRollers.getPIDController().setReference(voltage.magnitude(), ControlType.kVoltage); }
}
