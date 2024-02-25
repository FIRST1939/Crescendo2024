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

public class Shooter extends SubsystemBase {

    private CANSparkFlex topRollers;
    private CANSparkFlex bottomRollers;

    public Shooter () {

        this.topRollers = new CANSparkFlex(Constants.ShooterConstants.TOP_ROLLERS, MotorType.kBrushless);
        this.bottomRollers = new CANSparkFlex(Constants.ShooterConstants.BOTTOM_ROLLERS, MotorType.kBrushless);

        this.topRollers.setInverted(Constants.ShooterConstants.TOP_ROLLERS_INVERTED);
        this.bottomRollers.setInverted(Constants.ShooterConstants.BOTTOM_ROLLERS_INVERTED);

        this.topRollers.getEncoder().setPositionConversionFactor(Constants.ShooterConstants.TOP_ROLLERS_REDUCTION * (Math.PI * Constants.ShooterConstants.TOP_ROLLERS_DIAMETER));
        this.topRollers.getEncoder().setVelocityConversionFactor((Constants.ShooterConstants.TOP_ROLLERS_REDUCTION / 60.0) * (Math.PI * Constants.ShooterConstants.TOP_ROLLERS_DIAMETER));

        this.bottomRollers.getEncoder().setPositionConversionFactor(Constants.ShooterConstants.BOTTOM_ROLLERS_REDUCTION * (Math.PI * Constants.ShooterConstants.BOTTOM_ROLLERS_DIAMETER));
        this.bottomRollers.getEncoder().setVelocityConversionFactor((Constants.ShooterConstants.BOTTOM_ROLLERS_REDUCTION / 60.0) * (Math.PI * Constants.ShooterConstants.BOTTOM_ROLLERS_DIAMETER));
    }

    public void setVelocity (double velocity) {

        double topMax = 6784 * Constants.ShooterConstants.TOP_ROLLERS_REDUCTION * (Math.PI * Constants.ShooterConstants.TOP_ROLLERS_DIAMETER) * (1 / 60.0);
        double bottomMax = 6784 * Constants.ShooterConstants.BOTTOM_ROLLERS_REDUCTION * (Math.PI * Constants.ShooterConstants.BOTTOM_ROLLERS_DIAMETER) * (1 / 60.0);

        this.topRollers.set(velocity / topMax);
        this.bottomRollers.set(velocity / bottomMax);
    }

    public boolean atSpeed () { 
        
        boolean topRollersAtSpeed = Math.abs(this.topRollers.getEncoder().getVelocity() - Constants.ShooterConstants.SHOOT_SPEED) < Constants.ShooterConstants.SHOOT_TOLERANCE;
        boolean bottomRollersAtSpeed = Math.abs(this.bottomRollers.getEncoder().getVelocity() - Constants.ShooterConstants.SHOOT_SPEED) < Constants.ShooterConstants.SHOOT_TOLERANCE;
        return topRollersAtSpeed && bottomRollersAtSpeed;
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
                        .linearPosition(Units.Inches.of(this.topRollers.getEncoder().getPosition()))
                        .linearVelocity(Units.InchesPerSecond.of(this.topRollers.getEncoder().getVelocity()))
                        .voltage(Units.Volts.of(this.topRollers.getBusVoltage() * this.topRollers.getAppliedOutput()));

                    sysIdRoutineLog.motor("shooter-bottom-rollers")
                        .linearPosition(Units.Inches.of(this.bottomRollers.getEncoder().getPosition()))
                        .linearVelocity(Units.InchesPerSecond.of(this.bottomRollers.getEncoder().getVelocity()))
                        .voltage(Units.Volts.of(this.bottomRollers.getBusVoltage() * this.bottomRollers.getAppliedOutput()));
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
