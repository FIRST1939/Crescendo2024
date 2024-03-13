package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.ThroughBoreEncoder;
import frc.robot.util.Constants;
import frc.robot.util.Constants.IdleBehavior;

public class Elevator extends SubsystemBase {
    
    private TalonFX leadRaise;
    private TalonFX followerRaise;

    private ThroughBoreEncoder raiseEncoder;
    private DoubleSupplier raisePosition;

    private DigitalInput lowerBound;
    private DigitalInput upperBound;

    public Elevator () {

        this.leadRaise = new TalonFX(Constants.ElevatorConstants.LEAD_RAISE);
        this.followerRaise = new TalonFX(Constants.ElevatorConstants.FOLLOWER_RAISE);

        this.raiseEncoder = new ThroughBoreEncoder(Constants.ElevatorConstants.RAISE_ENCODER);
        this.raisePosition = () -> (this.raiseEncoder.get() - Constants.ElevatorConstants.RAISE_OFFSET);

        this.lowerBound = new DigitalInput(Constants.ElevatorConstants.LOWER_BOUND);
        this.upperBound = new DigitalInput(Constants.ElevatorConstants.UPPER_BOUND);
    }

    @Override
    public void periodic () {

        this.raiseEncoder.poll();
    }

    public void setPosition (double position) {}

    public void setVelocity (double velocity) {

        if (velocity < 0.0 && this.lowerBound.get()) velocity = 0.0;
        if (velocity > 0.0 && this.upperBound.get()) velocity = 0.0;
        this.leadRaise.set(velocity);
        this.followerRaise.set(velocity);
    }

    public double getPosition () { return this.raisePosition.getAsDouble(); }
    public boolean atPosition () { return false; }

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
