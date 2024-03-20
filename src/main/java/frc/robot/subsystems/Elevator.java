package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.ThroughBoreEncoder;
import frc.robot.util.Constants;
import frc.robot.util.Constants.IdleBehavior;

public class Elevator extends SubsystemBase {
    
    private TalonFX leadRaise;
    private TalonFX followerRaise;

    private PIDController raiseController;
    private VoltageOut voltageOut = new VoltageOut(0);

    private ThroughBoreEncoder raiseEncoder;
    private DoubleSupplier raisePosition;

    private DigitalInput lowerBound;
    private DigitalInput upperBound;

    public Elevator () {

        this.leadRaise = new TalonFX(Constants.ElevatorConstants.LEAD_RAISE);
        this.followerRaise = new TalonFX(Constants.ElevatorConstants.FOLLOWER_RAISE);

        this.raiseController = new PIDController(
            Constants.ElevatorConstants.RAISE_P,
            0.0,
            0.0
        );

        this.raiseController.setTolerance(Constants.ElevatorConstants.RAISE_TOLERANCE);

        this.raiseEncoder = new ThroughBoreEncoder(Constants.ElevatorConstants.RAISE_ENCODER);
        this.raisePosition = () -> -(this.raiseEncoder.get() - Constants.ElevatorConstants.RAISE_OFFSET);

        this.lowerBound = new DigitalInput(Constants.ElevatorConstants.LOWER_BOUND);
        this.upperBound = new DigitalInput(Constants.ElevatorConstants.UPPER_BOUND);
    }

    @Override
    public void periodic () {

        this.raiseEncoder.poll();
    }

    public void setPosition (double position) {

        double error = position - this.raisePosition.getAsDouble();
        double feedforward = Constants.ElevatorConstants.RAISE_FF * Math.signum(error);
        double input = feedforward + this.raiseController.calculate(this.raisePosition.getAsDouble(), position);

        if (input < 0.0 && this.lowerBound.get()) { input = 0.0; }
        if (input > 0.0 && this.upperBound.get()) { input = 0.0; }
        input = Math.signum(input) * Math.min(Math.abs(input), Constants.ElevatorConstants.RAISE_CAP);

        this.leadRaise.setControl(this.voltageOut.withOutput(input));
        this.followerRaise.setControl(this.voltageOut.withOutput(input));
    }

    public void setVelocity (double velocity) {

        if (velocity < 0.0 && this.lowerBound.get()) velocity = 0.0;
        if (velocity > 0.0 && this.upperBound.get()) velocity = 0.0;
        this.leadRaise.set(velocity);
        this.followerRaise.set(velocity);
    }

    public double getPosition () { return this.raisePosition.getAsDouble(); }
    public boolean atPosition () { return this.raiseController.atSetpoint(); }

    public void setIdleBehavior (IdleBehavior idleBehavior) {

        if (idleBehavior == IdleBehavior.COAST) { 
            
            this.leadRaise.setNeutralMode(NeutralModeValue.Coast); 
            this.followerRaise.setNeutralMode(NeutralModeValue.Coast);
        } else if (idleBehavior == IdleBehavior.BRAKE) { 
            
            this.leadRaise.setNeutralMode(NeutralModeValue.Brake); 
            this.followerRaise.setNeutralMode(NeutralModeValue.Brake);
        }
    }
}
