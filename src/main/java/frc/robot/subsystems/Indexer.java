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

    private CANSparkMax topRoller;
    private CANSparkMax bottomRoller;
    private DigitalInput startBeam;
    private DigitalInput endBeam;

    public Indexer () {

        this.topRoller = new CANSparkMax(Constants.IndexerConstants.TOP_ROLLER, MotorType.kBrushless);
        this.bottomRoller = new CANSparkMax(Constants.IndexerConstants.BOTTOM_ROLLER, MotorType.kBrushless);

        this.topRoller.setInverted(Constants.IndexerConstants.TOP_ROLLER_INVERTED);
        this.bottomRoller.setInverted(Constants.IndexerConstants.BOTTOM_ROLLER_INVERTED);

        this.topRoller.getEncoder().setPositionConversionFactor(Constants.IndexerConstants.TOP_ROLLER_REDUCTION);
        this.topRoller.getEncoder().setVelocityConversionFactor(Constants.IndexerConstants.TOP_ROLLER_REDUCTION);

        this.bottomRoller.getEncoder().setPositionConversionFactor(Constants.IndexerConstants.BOTTOM_ROLLER_REDUCTION);
        this.bottomRoller.getEncoder().setVelocityConversionFactor(Constants.IndexerConstants.BOTTOM_ROLLER_REDUCTION);

        this.topRoller.getPIDController().setP(Constants.IndexerConstants.ROLLER_P);
        this.topRoller.getPIDController().setI(Constants.IndexerConstants.ROLLER_I);
        this.topRoller.getPIDController().setD(Constants.IndexerConstants.ROLLER_D);

        this.bottomRoller.getPIDController().setP(Constants.IndexerConstants.ROLLER_P);
        this.bottomRoller.getPIDController().setI(Constants.IndexerConstants.ROLLER_I);
        this.bottomRoller.getPIDController().setD(Constants.IndexerConstants.ROLLER_D);
    }

    public void setVelocity (double velocity) {

        this.topRoller.getPIDController().setReference(velocity, ControlType.kVelocity);
        this.bottomRoller.getPIDController().setReference(velocity, ControlType.kVelocity);
    }

    public boolean noteContained () { return false; }
    public boolean noteIndexed () { return false; }
    public boolean noteFed () { return false; }

    public Command getQuasistaticRoutine (Direction direction) { return this.getSysIdRoutine().quasistatic(direction); }
    public Command getDynamicRoutine (Direction direction) { return this.getSysIdRoutine().dynamic(direction); }

    private SysIdRoutine getSysIdRoutine () {

        return new SysIdRoutine(
            Constants.IndexerConstants.SYSID_ROUTINE_CONFIG,
            new Mechanism(
                this::setRollerVoltage,
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

    private void setRollerVoltage (Measure<Voltage> voltage) {

        this.topRoller.getPIDController().setReference(voltage.magnitude(), ControlType.kVoltage);
        this.bottomRoller.getPIDController().setReference(voltage.magnitude(), ControlType.kVoltage);
    }
}
