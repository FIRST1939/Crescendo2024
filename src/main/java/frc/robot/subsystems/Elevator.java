package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.ThroughBoreEncoder;
import frc.robot.util.Constants;
import frc.robot.util.Constants.IdleBehavior;
import frc.robot.util.Sensors;

public class Elevator extends SubsystemBase {
    
    private TalonFX leadRaise;
    private TalonFX followerRaise;

    private PIDController raiseController;
    private VoltageOut voltageOut = new VoltageOut(0);

    private ThroughBoreEncoder raiseEncoder;
    private DoubleSupplier raisePosition;

    public Elevator () {

        this.leadRaise = new TalonFX(Constants.ElevatorConstants.LEAD_RAISE);
        this.followerRaise = new TalonFX(Constants.ElevatorConstants.FOLLOWER_RAISE);

        this.leadRaise.setInverted(Constants.ElevatorConstants.LEAD_RAISE_INVERTED);
        this.followerRaise.setInverted(Constants.ElevatorConstants.FOLLOWER_RAISE_INVERTED);

        this.raiseController = new PIDController(
            Constants.ElevatorConstants.RAISE_KP,
            0.0,
            0.0
        );

        this.raiseController.setTolerance(Constants.ElevatorConstants.RAISE_TOLERANCE);

        this.raiseEncoder = new ThroughBoreEncoder(Constants.ElevatorConstants.RAISE_ENCODER);
        this.raisePosition = () -> -(this.raiseEncoder.get() - Constants.ElevatorConstants.RAISE_OFFSET);
    }

    @Override
    public void periodic () {

        this.raiseEncoder.poll();

        SmartDashboard.putBoolean("Elevator Lower Bound", Sensors.getElevatorLowerBound());
        SmartDashboard.putBoolean("Elevator Upper Bound", Sensors.getElevatorUpperBound());
        
        SmartDashboard.putNumber("Elevator Position", this.raisePosition.getAsDouble());
        SmartDashboard.putNumber("Elevator Error", this.raiseController.getPositionError());
    }

    public void setPosition (double position) {

        double error = position - this.raisePosition.getAsDouble();
        double feedforward = Constants.ElevatorConstants.RAISE_KS * Math.signum(error);
        double input = feedforward + this.raiseController.calculate(this.raisePosition.getAsDouble(), position);

        if (input < 0.0 && Sensors.getElevatorLowerBound()) { input = 0.0; }
        if (input > 0.0 && Sensors.getElevatorUpperBound()) { input = 0.0; }
        input = Math.signum(input) * Math.min(Math.abs(input), Constants.ElevatorConstants.RAISE_CAP);

        //this.leadRaise.setControl(this.voltageOut.withOutput(input));
        //this.followerRaise.setControl(this.voltageOut.withOutput(input));

        if (position != 0.0) { this.setInput(0.45); }
        else { this.setInput(-0.45); }
    }

    public void setInput (double input) {

        if (input < 0.0 && Sensors.getElevatorLowerBound()) { input = 0.0; }
        if (input > 0.0 && Sensors.getElevatorUpperBound()) { input = 0.0; }
        this.leadRaise.set(input);
        this.followerRaise.set(input);
    }

    public double getPosition () { return this.raisePosition.getAsDouble(); }
    public boolean atHeight () { 
        
        //return (this.raiseController.atSetpoint() || Sensors.getElevatorUpperBound()); 
        return (Sensors.getElevatorLowerBound() || Sensors.getElevatorUpperBound());
    }

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
