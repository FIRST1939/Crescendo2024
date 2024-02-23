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
    private CANSparkMax backRollers;
    private DigitalInput startBeam;
    private DigitalInput endBeam;

    public Indexer () {

        this.frontRollers = new CANSparkMax(Constants.IndexerConstants.FRONT_ROLLERS, MotorType.kBrushless);
        this.backRollers = new CANSparkMax(Constants.IndexerConstants.BACK_ROLLERS, MotorType.kBrushless);

        this.frontRollers.setInverted(Constants.IndexerConstants.FRONT_ROLLERS_INVERTED);
        this.backRollers.setInverted(Constants.IndexerConstants.BACK_ROLLERS_INVERTED);

        this.frontRollers.getEncoder().setPositionConversionFactor(Constants.IndexerConstants.FRONT_ROLLERS_REDUCTION);
        this.frontRollers.getEncoder().setVelocityConversionFactor(Constants.IndexerConstants.FRONT_ROLLERS_REDUCTION);

        this.backRollers.getEncoder().setPositionConversionFactor(Constants.IndexerConstants.BACK_ROLLERS_REDUCTION);
        this.backRollers.getEncoder().setVelocityConversionFactor(Constants.IndexerConstants.BACK_ROLLERS_REDUCTION);

        this.frontRollers.getPIDController().setP(Constants.IndexerConstants.FRONT_ROLLERS_P);
        this.frontRollers.getPIDController().setI(Constants.IndexerConstants.FRONT_ROLLERS_I);
        this.frontRollers.getPIDController().setD(Constants.IndexerConstants.FRONT_ROLLERS_D);

        this.backRollers.getPIDController().setP(Constants.IndexerConstants.BACK_ROLLERS_P);
        this.backRollers.getPIDController().setI(Constants.IndexerConstants.BACK_ROLLERS_I);
        this.backRollers.getPIDController().setD(Constants.IndexerConstants.BACK_ROLLERS_D);

        this.startBeam = new DigitalInput(Constants.IndexerConstants.START_BEAM);
        this.endBeam = new DigitalInput(Constants.IndexerConstants.END_BEAM);
    }

    public void setVelocity (double velocity) {

        double frontMax = 6784 * Constants.IndexerConstants.FRONT_ROLLERS_REDUCTION * (Math.PI * Constants.IndexerConstants.FRONT_ROLLERS_DIAMETER) * (1 / 60.0);
        double backMax = 11710 * Constants.IndexerConstants.BACK_ROLLERS_REDUCTION * (Math.PI * Constants.IndexerConstants.BACK_ROLLERS_DIAMETER) * (1 / 60.0);


        this.frontRollers.set(velocity / frontMax);
        this.backRollers.set(velocity / backMax);
    }

    public boolean noteContained () { return !this.startBeam.get(); }
    public boolean noteIndexed () { return !this.endBeam.get(); }
    public boolean noteFed () { return this.endBeam.get(); }

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

                    sysIdRoutineLog.motor("indexer-back-rollers")
                        .linearPosition(Units.Inches.of(this.backRollers.getEncoder().getPosition() * (Math.PI * Constants.IndexerConstants.BACK_ROLLERS_DIAMETER)))
                        .linearVelocity(Units.InchesPerSecond.of((this.backRollers.getEncoder().getVelocity() / 60.0) * (Math.PI * Constants.IndexerConstants.BACK_ROLLERS_DIAMETER)))
                        .voltage(Units.Volts.of(this.backRollers.getBusVoltage() * this.backRollers.getAppliedOutput()))
                        .current(Units.Amps.of(this.backRollers.getOutputCurrent()));
                },
                this
            )
        );
    }

    private void setFrontRollersVoltage (Measure<Voltage> voltage) { this.frontRollers.getPIDController().setReference(voltage.magnitude(), ControlType.kVoltage); }
    private void setBackRollersVoltage (Measure<Voltage> voltage) { this.backRollers.getPIDController().setReference(voltage.magnitude(), ControlType.kVoltage); }
}
