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
                  .linearPosition(Units.Inches.of(this.topRoller.getEncoder().getPosition()))
                  .linearVelocity(Units.InchesPerSecond.of(this.topRoller.getEncoder().getVelocity()))
                  .voltage(Units.Volts.of(this.topRoller.getBusVoltage() * this.topRoller.getAppliedOutput()));
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
                  .linearPosition(Units.Inches.of(this.bottomRoller.getEncoder().getPosition()))
                  .linearVelocity(Units.InchesPerSecond.of(this.bottomRoller.getEncoder().getVelocity()))
                  .voltage(Units.Volts.of(this.bottomRoller.getBusVoltage() * this.bottomRoller.getAppliedOutput()));
            },
            this
         )
      );
   }

   private void setTopRollerVoltage (Measure<Voltage> voltage) { this.topRoller.getPIDController().setReference(voltage.magnitude(), ControlType.kVoltage); }
   private void setBottomRollerVoltage (Measure<Voltage> voltage) { this.bottomRoller.getPIDController().setReference(voltage.magnitude(), ControlType.kVoltage); }
}
