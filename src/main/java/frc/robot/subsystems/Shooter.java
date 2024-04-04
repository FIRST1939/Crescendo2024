package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;
import frc.robot.util.Constants.IdleBehavior;

public class Shooter extends SubsystemBase {

    private CANSparkFlex topRollers;
    private CANSparkFlex bottomRollers;

    private PIDController topController;
    private PIDController bottomController;

    private double shooterSpeed;

    public Shooter () {

        this.topRollers = new CANSparkFlex(Constants.ShooterConstants.TOP_ROLLERS, MotorType.kBrushless);
        this.bottomRollers = new CANSparkFlex(Constants.ShooterConstants.BOTTOM_ROLLERS, MotorType.kBrushless);

        this.topRollers.setInverted(Constants.ShooterConstants.TOP_ROLLERS_INVERTED);
        this.bottomRollers.setInverted(Constants.ShooterConstants.BOTTOM_ROLLERS_INVERTED);

        this.topRollers.getEncoder().setPositionConversionFactor(Constants.ShooterConstants.TOP_ROLLERS_REDUCTION * (Math.PI * Constants.ShooterConstants.TOP_ROLLERS_DIAMETER));
        this.topRollers.getEncoder().setVelocityConversionFactor((Constants.ShooterConstants.TOP_ROLLERS_REDUCTION / 60.0) * (Math.PI * Constants.ShooterConstants.TOP_ROLLERS_DIAMETER));

        this.bottomRollers.getEncoder().setPositionConversionFactor(Constants.ShooterConstants.BOTTOM_ROLLERS_REDUCTION * (Math.PI * Constants.ShooterConstants.BOTTOM_ROLLERS_DIAMETER));
        this.bottomRollers.getEncoder().setVelocityConversionFactor((Constants.ShooterConstants.BOTTOM_ROLLERS_REDUCTION / 60.0) * (Math.PI * Constants.ShooterConstants.BOTTOM_ROLLERS_DIAMETER));

        this.topController = new PIDController(
            Constants.ShooterConstants.SPEED_KP,
            0.0,
            0.0
        );

        this.bottomController = new PIDController(
            Constants.ShooterConstants.SPEED_KP,
            0.0,
            0.0
        );

        this.topController.setTolerance(Constants.ShooterConstants.SPEED_TOLERANCE);
        this.bottomController.setTolerance(Constants.ShooterConstants.SPEED_TOLERANCE);
    }

    @Override
    public void periodic () {

        double topError = this.topController.getPositionError();
        double bottomError = this.bottomController.getPositionError();
        SmartDashboard.putNumber("Shooter Error", Math.abs((topError + bottomError) / 2.0));
    }

    public void setSpeed (double shooterSpeed) { this.shooterSpeed = shooterSpeed; }
    public double getSpeed () { return this.shooterSpeed; }

    public void setTopVelocity (double velocity) {

        if (velocity == 0.0) {

            this.topRollers.setVoltage(0.0);
            return;
        }

        double feedforward = Constants.ShooterConstants.SPEED_KS + (Constants.ShooterConstants.SPEED_KV * velocity);
        double input = feedforward + this.topController.calculate(this.getTopVelocity(), velocity);
        this.topRollers.setVoltage(input);
    }

    public void setBottomVelocity (double velocity) {

        if (velocity == 0.0) {

            this.bottomRollers.setVoltage(0.0);
            return;
        }

        double feedforward = Constants.ShooterConstants.SPEED_KS + (Constants.ShooterConstants.SPEED_KV * velocity);
        double input = feedforward + this.bottomController.calculate(this.getBottomVelocity(), velocity);
        this.bottomRollers.setVoltage(input);
    }

    public double getTopVelocity () { return this.topRollers.getEncoder().getVelocity(); }
    public double getBottomVelocity () { return this.bottomRollers.getEncoder().getVelocity(); }

    public boolean atSpeed () { 
        
        return this.topController.atSetpoint() && this.bottomController.atSetpoint();
    }

    public void setIdleBehavior (IdleBehavior idleBehavior) {

        if (idleBehavior == IdleBehavior.COAST) {

            this.topRollers.setIdleMode(IdleMode.kCoast);
            this.bottomRollers.setIdleMode(IdleMode.kCoast);
        }
        else if (idleBehavior == IdleBehavior.BRAKE) {

            this.topRollers.setIdleMode(IdleMode.kBrake);
            this.bottomRollers.setIdleMode(IdleMode.kBrake);
        }
    }
}
