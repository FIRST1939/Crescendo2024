package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;
import frc.robot.util.Constants.IdleBehavior;

public class Arm extends SubsystemBase {
    
    private TalonFX pivot;
    private DutyCycleEncoder pivotEncoder;

    private PIDController pivotController;
    private VoltageOut voltageOut = new VoltageOut(0);

    private DoubleSupplier pivotPosition;
    private DigitalInput lowerBound;
    private DigitalInput upperBound;

    public double manualPivotAdjustment = 0.0;

    public Arm () {

        this.pivot = new TalonFX(Constants.ArmConstants.PIVOT);
        this.pivot.setInverted(Constants.ArmConstants.PIVOT_INVERTED);

        this.pivotEncoder = new DutyCycleEncoder(Constants.ArmConstants.PIVOT_ENCODER);
        this.pivotController = new PIDController(
            Constants.ArmConstants.PIVOT_P,
            0.0,
            0.0
        );

        this.pivotPosition = () -> -(this.pivotEncoder.getAbsolutePosition() - Constants.ArmConstants.PIVOT_OFFSET) * 360;
        this.lowerBound = new DigitalInput(Constants.ArmConstants.LOWER_BOUND);
        this.upperBound = new DigitalInput(Constants.ArmConstants.UPPER_BOUND);
    }

    public void setPosition (double position) { 

        double error = position - this.pivotPosition.getAsDouble();
        double feedforward = 0.15 * -Math.signum(error);
        double input = feedforward - this.pivotController.calculate(this.pivotPosition.getAsDouble(), position);

        if (input < 0.0 && this.lowerBound.get()) input = 0.0;
        if (input > 0.0 && this.upperBound.get()) input = 0.0;
        this.pivot.setControl(this.voltageOut.withOutput(input));
    }

    public double getPosition () { return this.pivotPosition.getAsDouble(); }
    public boolean atPosition () { return this.pivotController.atSetpoint(); }

    public void setIdleBehavior (IdleBehavior idleBehavior) {

        if (idleBehavior == IdleBehavior.COAST) { this.pivot.setNeutralMode(NeutralModeValue.Coast); }
        else if (idleBehavior == IdleBehavior.BRAKE) { this.pivot.setNeutralMode(NeutralModeValue.Brake); }
    }
}
