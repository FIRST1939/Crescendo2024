package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.util.Constants;

public class Indexer extends SubsystemBase {

    private CANSparkMax frontRollers;
    private CANSparkMax topRoller;
    private CANSparkMax bottomRoller;
    private DigitalInput startBeam;
    private DigitalInput endBeam;

    public Indexer () {

        this.frontRollers = new CANSparkMax(Constants.IndexerConstants.FRONT_ROLLERS, MotorType.kBrushless);
        this.topRoller = new CANSparkMax(Constants.IndexerConstants.TOP_ROLLER, MotorType.kBrushless);
        this.bottomRoller = new CANSparkMax(Constants.IndexerConstants.BOTTOM_ROLLER, MotorType.kBrushless);

        this.frontRollers.setInverted(Constants.IndexerConstants.FRONT_ROLLERS_INVERTED);
        this.topRoller.setInverted(Constants.IndexerConstants.TOP_ROLLER_INVERTED);
        this.bottomRoller.setInverted(Constants.IndexerConstants.BOTTOM_ROLLER_INVERTED);

        this.frontRollers.getEncoder().setPositionConversionFactor(Constants.IndexerConstants.FRONT_ROLLERS_REDUCTION);
        this.frontRollers.getEncoder().setVelocityConversionFactor(Constants.IndexerConstants.FRONT_ROLLERS_REDUCTION);

        this.topRoller.getEncoder().setPositionConversionFactor(Constants.IndexerConstants.TOP_ROLLER_REDUCTION);
        this.topRoller.getEncoder().setVelocityConversionFactor(Constants.IndexerConstants.TOP_ROLLER_REDUCTION);

        this.bottomRoller.getEncoder().setPositionConversionFactor(Constants.IndexerConstants.BOTTOM_ROLLER_REDUCTION);
        this.bottomRoller.getEncoder().setVelocityConversionFactor(Constants.IndexerConstants.BOTTOM_ROLLER_REDUCTION);

        this.frontRollers.getPIDController().setP(Constants.IndexerConstants.FRONT_ROLLERS_P);
        this.frontRollers.getPIDController().setI(Constants.IndexerConstants.FRONT_ROLLERS_I);
        this.frontRollers.getPIDController().setD(Constants.IndexerConstants.FRONT_ROLLERS_D);

        this.topRoller.getPIDController().setP(Constants.IndexerConstants.BACK_ROLLERS_P);
        this.topRoller.getPIDController().setI(Constants.IndexerConstants.BACK_ROLLERS_I);
        this.topRoller.getPIDController().setD(Constants.IndexerConstants.BACK_ROLLERS_D);

        this.bottomRoller.getPIDController().setP(Constants.IndexerConstants.BACK_ROLLERS_P);
        this.bottomRoller.getPIDController().setI(Constants.IndexerConstants.BACK_ROLLERS_I);
        this.bottomRoller.getPIDController().setD(Constants.IndexerConstants.BACK_ROLLERS_D);
    }

    public void setVelocity (double velocity) {

        this.frontRollers.getPIDController().setReference(velocity, ControlType.kVelocity);
        this.topRoller.getPIDController().setReference(velocity, ControlType.kVelocity);
        this.bottomRoller.getPIDController().setReference(velocity, ControlType.kVelocity);
    }

    public boolean noteContained () { return false; }
    public boolean noteIndexed () { return false; }
    public boolean noteFed () { return false; }

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
                        .linearPosition(Units.Inches.of(this.frontRollers.getEncoder().getPosition() * (Math.PI * Constants.IndexerConstants.FRONT_ROLLERS_DIAMETER)))
                        .linearVelocity(Units.InchesPerSecond.of((this.frontRollers.getEncoder().getVelocity() / 60.0) * (Math.PI * Constants.IndexerConstants.FRONT_ROLLERS_DIAMETER)))
                        .voltage(Units.Volts.of(this.frontRollers.getBusVoltage() * this.frontRollers.getAppliedOutput()))
                        .current(Units.Amps.of(this.frontRollers.getOutputCurrent()));
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

                    sysIdRoutineLog.motor("indexer-top-roller")
                        .linearPosition(Units.Inches.of(this.topRoller.getEncoder().getPosition() * (Math.PI * Constants.IndexerConstants.TOP_ROLLER_DIAMETER)))
                        .linearVelocity(Units.InchesPerSecond.of((this.topRoller.getEncoder().getVelocity() / 60.0) * (Math.PI * Constants.IndexerConstants.TOP_ROLLER_DIAMETER)))
                        .voltage(Units.Volts.of(this.topRoller.getBusVoltage() * this.topRoller.getAppliedOutput()))
                        .current(Units.Amps.of(this.topRoller.getOutputCurrent()));

                    sysIdRoutineLog.motor("indexer-bottom-roller")
                        .linearPosition(Units.Inches.of(this.bottomRoller.getEncoder().getPosition() * (Math.PI * Constants.IndexerConstants.BOTTOM_ROLLER_DIAMETER)))
                        .linearVelocity(Units.InchesPerSecond.of((this.bottomRoller.getEncoder().getVelocity() / 60.0) * (Math.PI * Constants.IndexerConstants.BOTTOM_ROLLER_DIAMETER)))
                        .voltage(Units.Volts.of(this.bottomRoller.getBusVoltage() * this.bottomRoller.getAppliedOutput()))
                        .current(Units.Amps.of(this.bottomRoller.getOutputCurrent()));
                },
                this
            )
        );
    }

    private void setFrontRollersVoltage (Measure<Voltage> voltage) { this.frontRollers.getPIDController().setReference(voltage.magnitude(), ControlType.kVoltage); }

    private void setBackRollersVoltage (Measure<Voltage> voltage) {

        this.topRoller.getPIDController().setReference(voltage.magnitude(), ControlType.kVoltage);
        this.bottomRoller.getPIDController().setReference(voltage.magnitude(), ControlType.kVoltage);
    }
}
