package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;
import frc.robot.util.Constants.IdleBehavior;

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

      this.topRoller.getPIDController().setP(Constants.IntakeConstants.TOP_ROLLER_P);
      this.topRoller.getPIDController().setI(Constants.IntakeConstants.TOP_ROLLER_I);
      this.topRoller.getPIDController().setD(Constants.IntakeConstants.TOP_ROLLER_D);

      this.bottomRoller.getPIDController().setP(Constants.IntakeConstants.BOTTOM_ROLLER_P);
      this.bottomRoller.getPIDController().setI(Constants.IntakeConstants.BOTTOM_ROLLER_I);
      this.bottomRoller.getPIDController().setD(Constants.IntakeConstants.BOTTOM_ROLLER_D);
   }

   public void setVelocity (double velocity) {

      double topMax = 5820 * Constants.IntakeConstants.TOP_ROLLER_REDUCTION * (Math.PI * Constants.IntakeConstants.TOP_ROLLER_DIAMETER) * (1 / 60.0);
      double bottomMax = 11710 * Constants.IntakeConstants.BOTTOM_ROLLER_REDUCTION * (Math.PI * Constants.IntakeConstants.BOTTOM_ROLLER_DIAMETER) * (1 / 60.0);

      this.topRoller.set(velocity / topMax);
      this.bottomRoller.set(velocity / bottomMax);
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
