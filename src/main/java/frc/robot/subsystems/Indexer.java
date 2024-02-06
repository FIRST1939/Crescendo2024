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

    private CANSparkMax topRollers;
    private CANSparkMax bottomRollers;
    private DigitalInput startBeam;
    private DigitalInput endBeam;

    public Indexer () {

        this.topRollers = new CANSparkMax(Constants.IndexerConstants.TOP_ROLLERS, MotorType.kBrushless);
        this.bottomRollers = new CANSparkMax(Constants.IndexerConstants.BOTTOM_ROLLERS, MotorType.kBrushless);

        this.topRollers.setInverted(Constants.IndexerConstants.TOP_ROLLERS_INVERTED);
        this.bottomRollers.setInverted(Constants.IndexerConstants.BOTTOM_ROLLERS_INVERTED);

        this.topRollers.getEncoder().setPositionConversionFactor(Constants.IndexerConstants.TOP_ROLLERS_REDUCTION);
        this.topRollers.getEncoder().setVelocityConversionFactor(Constants.IndexerConstants.TOP_ROLLERS_REDUCTION);

        this.bottomRollers.getEncoder().setPositionConversionFactor(Constants.IndexerConstants.BOTTOM_ROLLERS_REDUCTION);
        this.bottomRollers.getEncoder().setVelocityConversionFactor(Constants.IndexerConstants.BOTTOM_ROLLERS_REDUCTION);
    }

    public Command getQuasistaticRoutine (Direction direction) { return this.getSysIdRoutine().quasistatic(direction); }
    public Command getDynamicRoutine (Direction direction) { return this.getSysIdRoutine().dynamic(direction); }

    private SysIdRoutine getSysIdRoutine () {

        return new SysIdRoutine(
            Constants.IndexerConstants.SYSID_ROUTINE_CONFIG,
            new Mechanism(
                this::setRollersVoltage,
                sysIdRoutineLog -> {

                    sysIdRoutineLog.motor("indexer-top-rollers")
                        .angularPosition(Units.Rotations.of(this.topRollers.getEncoder().getPosition()))
                        .angularVelocity(Units.Rotations.of(this.topRollers.getEncoder().getVelocity()).per(Units.Second))
                        .linearPosition(Units.Inches.of(this.topRollers.getEncoder().getPosition() * (Math.PI * Constants.IndexerConstants.ROLLERS_DIAMETER)))
                        .linearVelocity(Units.InchesPerSecond.of(this.topRollers.getEncoder().getPosition() * (Math.PI * Constants.IndexerConstants.ROLLERS_DIAMETER)))
                        .voltage(Units.Volts.of(this.topRollers.getBusVoltage() * this.topRollers.getAppliedOutput()))
                        .current(Units.Amps.of(this.topRollers.getOutputCurrent()));

                    sysIdRoutineLog.motor("indexer-bottom-rollers")
                        .angularPosition(Units.Rotations.of(this.bottomRollers.getEncoder().getPosition()))
                        .angularVelocity(Units.Rotations.of(this.bottomRollers.getEncoder().getVelocity()).per(Units.Second))
                        .linearPosition(Units.Inches.of(this.bottomRollers.getEncoder().getPosition() * (Math.PI * Constants.IndexerConstants.ROLLERS_DIAMETER)))
                        .linearVelocity(Units.InchesPerSecond.of(this.bottomRollers.getEncoder().getPosition() * (Math.PI * Constants.IndexerConstants.ROLLERS_DIAMETER)))
                        .voltage(Units.Volts.of(this.bottomRollers.getBusVoltage() * this.bottomRollers.getAppliedOutput()))
                        .current(Units.Amps.of(this.bottomRollers.getOutputCurrent()));
                },
                this
            )
        );
    }

    private void setRollersVoltage (Measure<Voltage> voltage) {

        this.topRollers.getPIDController().setReference(voltage.magnitude(), ControlType.kVoltage);
        this.bottomRollers.getPIDController().setReference(voltage.magnitude(), ControlType.kVoltage);
    }
}
