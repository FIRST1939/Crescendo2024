package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;
import frc.robot.util.Constants.IdleBehavior;

public class Elevator extends SubsystemBase {
    
    private TalonFX leadElevation;
    private TalonFX followerElevation;

    private DutyCycleEncoder elevationEncoder;
    private PIDController elevationController;
    private Timer setpointTimer;

    private DoubleSupplier elevationPosition;
    private DigitalInput upperBound;

    public Elevator () {

        this.leadElevation = new TalonFX(Constants.ElevatorConstants.LEAD_ELEVATION);
        this.followerElevation = new TalonFX(Constants.ElevatorConstants.FOLLOWER_ELEVATION);

        this.elevationEncoder = new DutyCycleEncoder(Constants.ElevatorConstants.ELEVATION_ENCODER);
        this.elevationController = new PIDController(
            Constants.ElevatorConstants.ELEVATION_P,
            Constants.ElevatorConstants.ELEVATION_I,
            Constants.ElevatorConstants.ELEVATION_D
        );

        this.elevationController.setIZone(Constants.ElevatorConstants.ELEVATION_IZ);
        this.elevationController.setTolerance(Constants.ElevatorConstants.ELEVATION_TOLERANCE);

        this.setpointTimer = new Timer();

        this.elevationPosition = () -> 0.0;
        this.upperBound = new DigitalInput(Constants.ElevatorConstants.UPPER_BOUND);
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

        if (this.atPosition() && this.setpointTimer.get() > 0.5) { this.elevationController.setI(0); }
        else { this.elevationController.setI(Constants.ElevatorConstants.ELEVATION_I); }

        double input = this.elevationController.calculate(this.elevationPosition.getAsDouble(), position);
        if (this.setpointTimer.get() > 0.5 && Math.abs(input) < Constants.ElevatorConstants.INPUT_TOLERANCE) { input = 0.0; }
        if (input > 0.0 && this.upperBound.get()) input = 0.0;

        this.leadElevation.set(input);
        this.followerElevation.set(input);
    }

    public double getPosition () { return this.elevationPosition.getAsDouble(); }
    public boolean atPosition () { return this.elevationController.atSetpoint(); }

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
