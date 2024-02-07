package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.util.Constants;

public class Intake extends SubsystemBase {

   private CANSparkMax topRoller; 
   private CANSparkMax bottomRoller; 

   public Intake () {

      this.topRoller = new CANSparkMax(Constants.IntakeConstants.TOP_ROLLER, MotorType.kBrushless);
      this.bottomRoller = new CANSparkMax(Constants.IntakeConstants.BOTTOM_ROLLER, MotorType.kBrushless);

      this.topRoller.setInverted(Constants.IntakeConstants.TOP_ROLLER_INVERTED);
      this.bottomRoller.setInverted(Constants.IntakeConstants.BOTTOM_ROLLER_INVERTED);

      this.topRoller.getEncoder().setPositionConversionFactor(Constants.IntakeConstants.TOP_ROLLER_REDUCTION);
      this.topRoller.getEncoder().setVelocityConversionFactor(Constants.IntakeConstants.TOP_ROLLER_REDUCTION);

      this.bottomRoller.getEncoder().setPositionConversionFactor(Constants.IntakeConstants.BOTTOM_ROLLER_REDUCTION);
      this.bottomRoller.getEncoder().setVelocityConversionFactor(Constants.IntakeConstants.BOTTOM_ROLLER_REDUCTION);
   }

   public void setVelocity (double velocity) {

      this.topRoller.getPIDController().setReference(velocity, ControlType.kVelocity);
      this.bottomRoller.getPIDController().setReference(velocity, ControlType.kVelocity);
   }

   public Command getTopQuasistaticRoutine (Direction direction) { return this.getTopSysIdRoutine().quasistatic(direction); }
   public Command getTopDynamicRoutine (Direction direction) { return this.getTopSysIdRoutine().dynamic(direction); }

   public Command getBottomQuasistaticRoutine (Direction direction) { return this.getBottomSysIdRoutine().quasistatic(direction); }
   public Command getBottomDynamicRoutine (Direction direction) { return this.getBottomSysIdRoutine().dynamic(direction); }

   private SysIdRoutine getTopSysIdRoutine () {

      return new SysIdRoutine(
         Constants.IntakeConstants.TOP_SYSID_ROUTINE_CONFIG,
         new Mechanism(
            this::setTopRollerVoltage,
            sysIdRoutineLog -> {

               sysIdRoutineLog.motor("intake-top-roller")
                  .linearPosition(Units.Inches.of(this.topRoller.getEncoder().getPosition() * (Math.PI * Constants.IntakeConstants.TOP_ROLLER_DIAMETER)))
                  .linearVelocity(Units.InchesPerSecond.of((this.topRoller.getEncoder().getVelocity() / 60.0) * (Math.PI * Constants.IntakeConstants.TOP_ROLLER_DIAMETER)))
                  .voltage(Units.Volts.of(this.topRoller.getBusVoltage() * this.topRoller.getAppliedOutput()))
                  .current(Units.Amps.of(this.topRoller.getOutputCurrent()));
            },
            this
         )
      );
   }

   private SysIdRoutine getBottomSysIdRoutine () {

      return new SysIdRoutine(
         Constants.IntakeConstants.BOTTOM_SYSID_ROUTINE_CONFIG,
         new Mechanism(
            this::setBottomRollerVoltage,
            sysIdRoutineLog -> {

               sysIdRoutineLog.motor("intake-bottom-roller")
                  .linearPosition(Units.Inches.of(this.bottomRoller.getEncoder().getPosition() * (Math.PI * Constants.IntakeConstants.BOTTOM_ROLLER_DIAMETER)))
                  .linearVelocity(Units.InchesPerSecond.of((this.bottomRoller.getEncoder().getVelocity() / 60.0) * (Math.PI * Constants.IntakeConstants.BOTTOM_ROLLER_DIAMETER)))
                  .voltage(Units.Volts.of(this.topRoller.getBusVoltage() * this.topRoller.getAppliedOutput()))
                  .current(Units.Amps.of(this.bottomRoller.getOutputCurrent()));
            },
            this
         )
      );
   }

   private void setTopRollerVoltage (Measure<Voltage> voltage) { this.topRoller.getPIDController().setReference(voltage.magnitude(), ControlType.kVoltage); }
   private void setBottomRollerVoltage (Measure<Voltage> voltage) { this.bottomRoller.getPIDController().setReference(voltage.magnitude(), ControlType.kVoltage); }
}
