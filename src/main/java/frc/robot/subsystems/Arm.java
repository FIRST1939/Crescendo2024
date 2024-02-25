package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.util.Constants;

public class Arm extends SubsystemBase {
    
    private TalonFX pivot;
    private DutyCycleEncoder pivotEncoder;
    private PIDController pivotController;
    private Timer setpointTimer;

    private DoubleSupplier pivotPosition;
    private DoubleSupplier pivotVelocity;

    private DigitalInput lowerBound;
    private DigitalInput upperBound;

    public Arm () {

        this.pivot = new TalonFX(Constants.ArmConstants.PIVOT);
        this.pivot.setInverted(Constants.ArmConstants.PIVOT_INVERTED);

        this.pivotEncoder = new DutyCycleEncoder(Constants.ArmConstants.PIVOT_ENCODER);
        this.pivotEncoder.setPositionOffset(Constants.ArmConstants.PIVOT_OFFSET + this.pivotEncoder.getAbsolutePosition());

        this.pivotController = new PIDController(
            Constants.ArmConstants.PIVOT_P,
            Constants.ArmConstants.PIVOT_I,
            Constants.ArmConstants.PIVOT_D
        );

        this.pivotController.setIZone(Constants.ArmConstants.PIVOT_IZ);
        this.pivotController.setTolerance(Constants.ArmConstants.PIVOT_TOLERANCE);

        this.setpointTimer = new Timer();

        this.pivotPosition = () -> (-this.pivotEncoder.get() * 360) % 360;
        this.pivotVelocity = () -> this.pivot.getVelocity().getValue() * Constants.ArmConstants.PIVOT_REDUCTION * 360;

        this.lowerBound = new DigitalInput(Constants.ArmConstants.LOWER_BOUND);
        this.upperBound = new DigitalInput(Constants.ArmConstants.UPPER_BOUND);
    }

    @Override
    public void periodic () {

        if (this.atPosition() && this.setpointTimer.get() == 0.0) { this.setpointTimer.start(); }
        else if (!this.atPosition()) { 
            
            this.setpointTimer.stop(); 
            this.setpointTimer.reset();
        }
    }

    public void setPosition (double position) { 

        double input = -this.pivotController.calculate(this.pivotPosition.getAsDouble(), position);

        if (this.setpointTimer.get() > 0.5 && Math.abs(input) < Constants.ArmConstants.INPUT_TOLERANCE) { input = 0.0; }
        if (input < 0.0 && this.lowerBound.get()) input = 0.0;
        if (input > 0.0 && this.upperBound.get()) input = 0.0;

        this.pivot.set(input);
    }

    public boolean atPosition () { return this.pivotController.atSetpoint(); }

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
