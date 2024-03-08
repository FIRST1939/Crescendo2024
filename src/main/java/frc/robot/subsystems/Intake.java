package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;
import frc.robot.util.Constants.IdleBehavior;
import frc.robot.util.MotorUtil;

public class Intake extends SubsystemBase {

   private CANSparkMax topRoller; 
   private CANSparkMax bottomRoller; 

   public Intake () {

      this.topRoller = new CANSparkMax(Constants.IntakeConstants.TOP_ROLLER, MotorType.kBrushless);
      this.bottomRoller = new CANSparkMax(Constants.IntakeConstants.BOTTOM_ROLLER, MotorType.kBrushless);

      this.topRoller.setInverted(Constants.IntakeConstants.TOP_ROLLER_INVERTED);
      this.bottomRoller.setInverted(Constants.IntakeConstants.BOTTOM_ROLLER_INVERTED);

      this.topRoller.getEncoder().setPositionConversionFactor(Constants.IntakeConstants.TOP_ROLLER_REDUCTION * (Math.PI * Constants.IntakeConstants.TOP_ROLLER_DIAMETER));
      this.topRoller.getEncoder().setVelocityConversionFactor((Constants.IntakeConstants.TOP_ROLLER_REDUCTION / 60.0) * (Math.PI * Constants.IntakeConstants.TOP_ROLLER_DIAMETER));

      this.bottomRoller.getEncoder().setPositionConversionFactor(Constants.IntakeConstants.BOTTOM_ROLLER_REDUCTION * (Math.PI * Constants.IntakeConstants.BOTTOM_ROLLER_DIAMETER));
      this.bottomRoller.getEncoder().setVelocityConversionFactor((Constants.IntakeConstants.BOTTOM_ROLLER_REDUCTION / 60.0) * (Math.PI * Constants.IntakeConstants.BOTTOM_ROLLER_DIAMETER));
   }

   public void setVelocity (double velocity) {

      double topMaximumVelocity = MotorUtil.getMaxVelocity(
         frc.robot.util.MotorUtil.MotorType.NEO,
         Constants.IntakeConstants.TOP_ROLLER_DIAMETER,
         Constants.IntakeConstants.TOP_ROLLER_REDUCTION
      );

      double bottomMaximumVelocity = MotorUtil.getMaxVelocity(
         frc.robot.util.MotorUtil.MotorType.NEO550,
         Constants.IntakeConstants.BOTTOM_ROLLER_DIAMETER,
         Constants.IntakeConstants.BOTTOM_ROLLER_REDUCTION
      );

      this.topRoller.set(velocity / topMaximumVelocity);
      this.bottomRoller.set(velocity / bottomMaximumVelocity);
   }

   public void setIdleBehavior (IdleBehavior idleBehavior) {

      if (idleBehavior == IdleBehavior.COAST) {

         this.topRoller.setIdleMode(IdleMode.kCoast);
         this.bottomRoller.setIdleMode(IdleMode.kCoast);
      } else if (idleBehavior == IdleBehavior.BRAKE) {

         this.topRoller.setIdleMode(IdleMode.kBrake);
         this.bottomRoller.setIdleMode(IdleMode.kBrake);
      }
   }
}
