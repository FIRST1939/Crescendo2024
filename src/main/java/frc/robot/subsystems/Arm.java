package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.util.Constants;

public class Arm extends SubsystemBase {
    
    private TalonFX pivot;
    private DutyCycleEncoder pivotEncoder;

    private DoubleSupplier pivotPosition;
    private DoubleSupplier pivotVelocity;

    private DigitalInput lowerBound;
    private DigitalInput upperBound;

    public Arm () {

        this.pivot = new TalonFX(Constants.ArmConstants.PIVOT);
        this.pivot.setInverted(Constants.ArmConstants.PIVOT_INVERTED);

        this.pivotEncoder = new DutyCycleEncoder(Constants.ArmConstants.PIVOT_ENCODER);
        this.pivot.setPosition(this.pivotEncoder.getAbsolutePosition());

        this.pivot.getPosition().setUpdateFrequency(50);
        this.pivot.getVelocity().setUpdateFrequency(50);
        this.pivot.getMotorVoltage().setUpdateFrequency(50);
        this.pivot.getTorqueCurrent().setUpdateFrequency(50);

        this.pivotPosition = () -> this.pivot.getPosition().getValue() * Constants.ArmConstants.PIVOT_REDUCTION;
        this.pivotVelocity = () -> this.pivot.getVelocity().getValue() * Constants.ArmConstants.PIVOT_REDUCTION;
    }

    public void setPosition (double position) { this.pivot.setControl(new PositionVoltage(position)); }
    public boolean atPosition () { return false; }

    public Command getQuasistaticRoutine (Direction direction) { return this.getSysIdRoutine().quasistatic(direction); }
    public Command getDynamicRoutine (Direction direction) { return this.getSysIdRoutine().dynamic(direction); }

    private SysIdRoutine getSysIdRoutine () {

        return new SysIdRoutine(
            Constants.ArmConstants.SYSID_ROUTINE_CONFIG,
            new Mechanism(
                this::setPivotVoltage,
                sysIdRoutineLog -> {

                    sysIdRoutineLog.motor("arm-pivot")
                        .angularPosition(Units.Rotations.of(this.pivotPosition.getAsDouble()))
                        .angularVelocity(Units.Rotations.of(this.pivotVelocity.getAsDouble()).per(Units.Second))
                        .voltage(Units.Volts.of(this.pivot.getMotorVoltage().getValue()))
                        .current(Units.Amps.of(this.pivot.getTorqueCurrent().getValue()));
                },
                this
            )
        );
    }

    private void setPivotVoltage (Measure<Voltage> voltage) { this.pivot.setControl(new VoltageOut(voltage.magnitude())); }
}
