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
    
    private TalonFX leadElevation;
    private TalonFX followerElevation;

    private ThroughBoreEncoder elevationEncoder;
    private Timer setpointTimer;

    private DoubleSupplier elevationPosition;
    private DigitalInput lowerBound;
    private DigitalInput upperBound;

    public Elevator () {

        this.leadElevation = new TalonFX(Constants.ElevatorConstants.LEAD_ELEVATION);
        this.followerElevation = new TalonFX(Constants.ElevatorConstants.FOLLOWER_ELEVATION);

        this.elevationEncoder = new ThroughBoreEncoder(Constants.ElevatorConstants.ELEVATION_ENCODER);
        this.setpointTimer = new Timer();

        this.elevationPosition = () -> (this.elevationEncoder.get() - Constants.ElevatorConstants.ELEVATION_OFFSET) * 360;
        this.lowerBound = new DigitalInput(Constants.ElevatorConstants.LOWER_BOUND);
        this.upperBound = new DigitalInput(Constants.ElevatorConstants.UPPER_BOUND);
    }

    @Override
    public void periodic () {

        this.elevationEncoder.poll();

        if (this.atPosition() && this.setpointTimer.get() == 0.0) { this.setpointTimer.start(); }
        else if (!this.atPosition()) { 
            
            this.setpointTimer.stop(); 
            this.setpointTimer.reset();
        }
    }

    public void setPosition (double position) {}

    public double getPosition () { return this.elevationPosition.getAsDouble(); }
    public boolean atPosition () { return false; }

    public void setIdleBehavior (IdleBehavior idleBehavior) {

        if (idleBehavior == IdleBehavior.COAST) { 
            
            this.leadElevation.setNeutralMode(NeutralModeValue.Coast); 
            this.followerElevation.setNeutralMode(NeutralModeValue.Coast);
        } else if (idleBehavior == IdleBehavior.BRAKE) { 
            
            this.leadElevation.setNeutralMode(NeutralModeValue.Brake); 
            this.followerElevation.setNeutralMode(NeutralModeValue.Brake);
        }
    }
}
