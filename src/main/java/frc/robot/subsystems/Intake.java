package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;

public class Intake extends SubsystemBase {

   private CANSparkMax topRollers; 
   private CANSparkMax bottomRollers; 

   public Intake () {

      this.topRollers = new CANSparkMax(Constants.IntakeConstants.TOP_ROLLERS, MotorType.kBrushless);
      this.bottomRollers = new CANSparkMax(Constants.IntakeConstants.BOTTOM_ROLLERS, MotorType.kBrushless);

      this.topRollers.setInverted(Constants.IntakeConstants.TOP_ROLLERS_INVERTED);
      this.bottomRollers.setInverted(Constants.IntakeConstants.BOTTOM_ROLLERS_INVERTED);
   }
}
