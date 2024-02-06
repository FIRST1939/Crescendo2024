package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.util.Constants;

public class Shooter extends SubsystemBase{

    private CANSparkFlex topRollers;
    private CANSparkFlex bottomRollers;

    public Shooter () {

        this.topRollers = new CANSparkFlex(Constants.ShooterConstants.TOP_ROLLERS, MotorType.kBrushless);
        this.bottomRollers = new CANSparkFlex(Constants.ShooterConstants.BOTTOM_ROLLERS, MotorType.kBrushless);

        this.topRollers.setInverted(Constants.ShooterConstants.TOP_ROLLERS_INVERTED);
        this.bottomRollers.setInverted(Constants.ShooterConstants.BOTTOM_ROLLERS_INVERTED);

        this.topRollers.getEncoder().setPositionConversionFactor(Constants.ShooterConstants.TOP_ROLLERS_REDUCTION);
        this.topRollers.getEncoder().setVelocityConversionFactor(Constants.ShooterConstants.TOP_ROLLERS_REDUCTION);

        this.bottomRollers.getEncoder().setPositionConversionFactor(Constants.ShooterConstants.BOTTOM_ROLLERS_REDUCTION);
        this.bottomRollers.getEncoder().setVelocityConversionFactor(Constants.ShooterConstants.BOTTOM_ROLLERS_REDUCTION);
    }

    public Command getQuasistaticRoutine (Direction direction) { return this.getSysIdRoutine().quasistatic(direction); }
    public Command getDynamicRoutine (Direction direction) { return this.getSysIdRoutine().dynamic(direction); }

    private SysIdRoutine getSysIdRoutine () {

        return new SysIdRoutine(
            Constants.ShooterConstants.SYSID_ROUTINE_CONFIG,
            new Mechanism(
                this::setRollersVoltage,
                sysIdRoutineLog -> {

                    sysIdRoutineLog.motor("shooter-top-rollers")
                        .angularPosition(Units.Rotations.of(this.topRollers.getEncoder().getPosition()))
                        .angularVelocity(Units.Rotations.of(this.topRollers.getEncoder().getVelocity()).per(Units.Second))
                        .linearPosition(Units.Inches.of(this.topRollers.getEncoder().getPosition() * (Math.PI * Constants.ShooterConstants.ROLLERS_DIAMETER)))
                        .linearVelocity(Units.InchesPerSecond.of(this.topRollers.getEncoder().getPosition() * (Math.PI * Constants.ShooterConstants.ROLLERS_DIAMETER)))
                        .voltage(Units.Volts.of(this.topRollers.getBusVoltage() * this.topRollers.getAppliedOutput()))
                        .current(Units.Amps.of(this.topRollers.getOutputCurrent()));

                    sysIdRoutineLog.motor("shooter-bottom-rollers")
                        .angularPosition(Units.Rotations.of(this.bottomRollers.getEncoder().getPosition()))
                        .angularVelocity(Units.Rotations.of(this.bottomRollers.getEncoder().getVelocity()).per(Units.Second))
                        .linearPosition(Units.Inches.of(this.bottomRollers.getEncoder().getPosition() * (Math.PI * Constants.ShooterConstants.ROLLERS_DIAMETER)))
                        .linearVelocity(Units.InchesPerSecond.of(this.bottomRollers.getEncoder().getPosition() * (Math.PI * Constants.ShooterConstants.ROLLERS_DIAMETER)))
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
