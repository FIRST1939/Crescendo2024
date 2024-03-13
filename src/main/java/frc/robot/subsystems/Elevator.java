package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.ThroughBoreEncoder;
import frc.robot.util.Constants;
import frc.robot.util.Constants.IdleBehavior;

public class Elevator extends SubsystemBase {
    
    private TalonFX leadRaise;
    private TalonFX followerRaise;

    private ThroughBoreEncoder raiseEncoder;
    private Timer setpointTimer;

    private DoubleSupplier raisePosition;
    private DigitalInput lowerBound;
    private DigitalInput upperBound;

    public Elevator () {

        this.leadRaise = new TalonFX(Constants.ElevatorConstants.LEAD_RAISE);
        this.followerRaise = new TalonFX(Constants.ElevatorConstants.FOLLOWER_RAISE);

        this.raiseEncoder = new ThroughBoreEncoder(Constants.ElevatorConstants.RAISE_ENCODER);
        this.setpointTimer = new Timer();

        this.raisePosition = () -> (this.raiseEncoder.get() - Constants.ElevatorConstants.RAISE_OFFSET);
        this.lowerBound = new DigitalInput(Constants.ElevatorConstants.LOWER_BOUND);
        this.upperBound = new DigitalInput(Constants.ElevatorConstants.UPPER_BOUND);
    }

    @Override
    public void periodic () {

        this.raiseEncoder.poll();

        if (this.atPosition() && this.setpointTimer.get() == 0.0) { this.setpointTimer.start(); }
        else if (!this.atPosition()) { 
            
            this.setpointTimer.stop(); 
            this.setpointTimer.reset();
        }
    }

    public void setPosition (double position) {}

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
