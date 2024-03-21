package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;
import frc.robot.util.Constants.IdleBehavior;
import frc.robot.util.Sensors;

public class Arm extends SubsystemBase {
    
    private TalonFX pivot;
    private DutyCycleEncoder pivotEncoder;

    private PIDController pivotController;
    private DoubleSupplier pivotPosition;
    private VoltageOut voltageOut = new VoltageOut(0);

    public double manualPivotAdjustment = 0.0;

    public Arm () {

        this.pivot = new TalonFX(Constants.ArmConstants.PIVOT);
        this.pivot.setInverted(Constants.ArmConstants.PIVOT_INVERTED);

        this.pivotEncoder = new DutyCycleEncoder(Constants.ArmConstants.PIVOT_ENCODER);
        this.pivotController = new PIDController(
            Constants.ArmConstants.PIVOT_KP,
            0.0,
            0.0
        );

        this.pivotPosition = () -> -(this.pivotEncoder.getAbsolutePosition() - Constants.ArmConstants.PIVOT_OFFSET) * 360;
    }

    public void setPosition (double position) { 

        double error = position - this.pivotPosition.getAsDouble();
        double feedforward = Constants.ArmConstants.PIVOT_KS * -Math.signum(error);
        double input = feedforward - this.pivotController.calculate(this.pivotPosition.getAsDouble(), position);

        if (input < 0.0 && Sensors.getArmLowerBound()) input = 0.0;
        if (input > 0.0 && Sensors.getArmUpperBound()) input = 0.0;
        this.pivot.setControl(this.voltageOut.withOutput(input));
    }

    public double getPosition () { return this.pivotPosition.getAsDouble(); }
    public boolean atPosition () { return this.pivotController.atSetpoint(); }

    public void setIdleBehavior (IdleBehavior idleBehavior) {

        if (idleBehavior == IdleBehavior.COAST) { this.pivot.setNeutralMode(NeutralModeValue.Coast); }
        else if (idleBehavior == IdleBehavior.BRAKE) { this.pivot.setNeutralMode(NeutralModeValue.Brake); }
    }
}
