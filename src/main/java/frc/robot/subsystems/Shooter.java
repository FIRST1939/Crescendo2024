package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;
import frc.robot.util.Constants.IdleBehavior;

public class Shooter extends SubsystemBase {

    private CANSparkFlex topRollers;
    private CANSparkFlex bottomRollers;

    private BangBangController topBangBangController;
    private BangBangController bottomBangBangController;

    public Shooter () {

        this.topRollers = new CANSparkFlex(Constants.ShooterConstants.TOP_ROLLERS, MotorType.kBrushless);
        this.bottomRollers = new CANSparkFlex(Constants.ShooterConstants.BOTTOM_ROLLERS, MotorType.kBrushless);

        this.topRollers.setInverted(Constants.ShooterConstants.TOP_ROLLERS_INVERTED);
        this.bottomRollers.setInverted(Constants.ShooterConstants.BOTTOM_ROLLERS_INVERTED);

        this.topRollers.getEncoder().setPositionConversionFactor(Constants.ShooterConstants.TOP_ROLLERS_REDUCTION * (Math.PI * Constants.ShooterConstants.TOP_ROLLERS_DIAMETER));
        this.topRollers.getEncoder().setVelocityConversionFactor((Constants.ShooterConstants.TOP_ROLLERS_REDUCTION / 60.0) * (Math.PI * Constants.ShooterConstants.TOP_ROLLERS_DIAMETER));

        this.bottomRollers.getEncoder().setPositionConversionFactor(Constants.ShooterConstants.BOTTOM_ROLLERS_REDUCTION * (Math.PI * Constants.ShooterConstants.BOTTOM_ROLLERS_DIAMETER));
        this.bottomRollers.getEncoder().setVelocityConversionFactor((Constants.ShooterConstants.BOTTOM_ROLLERS_REDUCTION / 60.0) * (Math.PI * Constants.ShooterConstants.BOTTOM_ROLLERS_DIAMETER));

        this.topBangBangController = new BangBangController();
        this.topBangBangController.setTolerance(Constants.ShooterConstants.SHOOT_TOLERANCE);
        
        this.bottomBangBangController = new BangBangController();
        this.bottomBangBangController.setTolerance(Constants.ShooterConstants.SHOOT_TOLERANCE);
    }

    public void setTopVelocity (double velocity) {

        this.topBangBangController.setSetpoint(velocity);
        if (velocity == 0.0) { 
            this.topRollers.set(0);
            return; }

        this.topRollers.set(this.topBangBangController.calculate(this.getTopVelocity()));
    }

    public void setBottomVelocity (double velocity) {

        this.bottomBangBangController.setSetpoint(velocity);
        if (velocity == 0.0) { 
            this.bottomRollers.set(0);
            return; }

        this.bottomRollers.set(this.bottomBangBangController.calculate(this.getBottomVelocity()));
    }

    public double getTopVelocity () { return this.topRollers.getEncoder().getVelocity(); }
    public double getBottomVelocity () { return this.bottomRollers.getEncoder().getVelocity(); }

    public boolean atSpeed () { 
        
        return this.topBangBangController.atSetpoint() && this.bottomBangBangController.atSetpoint();
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
