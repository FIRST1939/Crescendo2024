package frc.robot.util;

import java.util.List;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import swervelib.math.Matter;

public final class Constants {
    
    public final class RobotConstants {

        public static final double ROBOT_MASS = 35.4626; // Robot Weight [kg]
        public static final double LOOP_TIME = 0.13; // Control Loop Tick Timing [s]

        private static final Matter BASE = new Matter(new Translation3d(-0.0057, -0.0037, 0.0813), 21.9116); // Weight Distribution of Base
        private static final Matter BATTERY = new Matter(new Translation3d(-0.2286, 0, 0.1225), 5.8468); // Weight Distribution of Battery
        private static final Matter BUMPERS = new Matter(new Translation3d(), 6.8039); // Weight Distribution of Bumpers

        public static final List<Matter> WEIGHT_DISTRIBUTION = List.of(BASE, BATTERY, BUMPERS);
    }

    public final class SwerveConstants {

        public static final IdleBehavior DISABLED_IDLE_BEHAVIOR = IdleBehavior.COAST; // While idle and disabled, the swerve drive is set to coast mode
        public static final IdleBehavior ENABLED_IDLE_BEHAVIOR = IdleBehavior.BRAKE; // While idle ad eabled, the swerve drive is set to brake mode

        private static final int FREE_SPEED = 5676; // Motor Free Speed [RPM]
        private static final double WHEEL_DIAMETER = 0.10033; // Diameter of the Wheel [m]

        private static final double DRIVE_REDUCTION = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0); // Gear Reduction of the Driving Motors [Wheel Revolution / Motor Revolution]
        private static final double STEER_REDUCTION = (7.0 / 150.0); // Gear Reduction of the Steering Motors [Wheel Revolution / Motor Revolution]

        public static final double MAX_DRIVE_SPEED = FREE_SPEED * DRIVE_REDUCTION * (WHEEL_DIAMETER * Math.PI) / 60.0; // Maximum Driving Speed [m / s]
        public static final double MAX_STEER_SPEED = FREE_SPEED * STEER_REDUCTION * (2 * Math.PI) / 60.0; // Maximum Steering Speed [rad / s]
    
        public static final double TRANSLATION_DEADBAND = 0.0825; // Joystick Deadband [%]
        public static final double OMEGA_DEADBAND = 0.125; // Joystick Deadband [%]

        public static final double TURN_CONSTANT = 0.225; // Turning Power [%]
        public static final double LOCK_TIME = 10.0; // Lock Time While Disabled [s]

        public static final double REPLANNING_TOTAL_ERROR = 0.35; // Total Error to Replan Path [m]
        public static final double REPLANNING_ERROR_SPIKE = 0.25; // Spike in Error to Replan Path [m / 20ms]

        public static final Config DRIVE_SYSID_CONFIG = new Config(
            Units.Volts.of(1.5).per(Units.Second),
            Units.Volts.of(9.6),
            Units.Seconds.of(10)
        );

        public static final Config ANGLE_SYSID_CONFIG = new Config(
            Units.Volts.of(1.25).per(Units.Second),
            Units.Volts.of(7.8),
            Units.Seconds.of(10)
        );

        public static final double DRIVE_SYSID_QUASISTATIC_TIMEOUT = 7.5; 
        public static final double DRIVE_SYSID_DYNAMIC_TIMEOUT = 3.0;

        public static final double ANGLE_SYSID_QUASISTATIC_TIMEOUT = 8.0;
        public static final double ANGLE_SYSID_DYNAMIC_TIMEOUT = 3.0;
    }

    public final class IntakeConstants {

        public static final int TOP_ROLLER = 31; // Sets the CANBUS ID of the Top Roller of the Intake [CANBUS]
        public static final int BOTTOM_ROLLER = 32; // Sets the CANBUS ID of the Bottom Roller of the Intake [CANBUS]

        public static final boolean TOP_ROLLER_INVERTED = false; // Sets the Intake Top Roller to spin forward depending on true/false
        public static final boolean BOTTOM_ROLLER_INVERTED = false; // Sets the Intake Bottom Roller to spin forward/backward depeding on true/false

        public static final IdleBehavior DISABLED_IDLE_BEHAVIOR = IdleBehavior.COAST; // While Disabled, the Intake is in Coast mode
        public static final IdleBehavior ENABLED_IDLE_BEHAVIOR = IdleBehavior.BRAKE; // While Enabled, the Intake is in Brake mode

        public static final double TOP_ROLLER_REDUCTION = (15.0 / 30.0); // The gear reduction for the top rollers [teeth over teeth]
        public static final double BOTTOM_ROLLER_REDUCTION = (1.0 / 2.89); // The gear reduction for bottom rollers [teeth over teeth]

        public static final double TOP_ROLLER_DIAMETER = 2.0; // Sets the diameter of the Top Roller [inches]
        public static final double BOTTOM_ROLLER_DIAMETER = 1.625; // Sets the diameter of the Bottom Roller [inches]

        public static final double TOP_ROLLER_P = 0.025;
        public static final double TOP_ROLLER_I = 0.0;
        public static final double TOP_ROLLER_D = 0.0;

        public static final double BOTTOM_ROLLER_P = 0.025;
        public static final double BOTTOM_ROLLER_I = 0.0;
        public static final double BOTTOM_ROLLER_D = 0.0;

        public static final double INTAKE_SPEED = 150.0; // Sets the Intake Speed [Inches per s]
        public static final double OUTAKE_SPEED = 0.0; // Sets the Outtake Speed [Inches per s]

        public static final Config TOP_SYSID_ROUTINE_CONFIG = new Config(
            Units.Volts.of(1.75).per(Units.Second), 
            Units.Volts.of(10),
            Units.Seconds.of(10)
        );

        public static final Config BOTTOM_SYSID_ROUTINE_CONFIG = new Config(
            Units.Volts.of(1.5).per(Units.Second), 
            Units.Volts.of(7),
            Units.Seconds.of(10)
        );
    }

    public final class IndexerConstants {

        public static final int FRONT_ROLLERS = 33; // Sets the CANBUS ID of the Indexer Front Rollers [CANBUS]
        public static final int BACK_ROLLERS = 34; // Sets the CANBUS ID of the Indexer Back Rollers [CANBUS]

        public static final boolean FRONT_ROLLERS_INVERTED = false; // Makes the Indexer Front Rollers spin forward/backward depending on false/true
        public static final boolean BACK_ROLLERS_INVERTED = false; // Makes the Indexer Back Rollers spin forward/backward depending on false/true

        public static final IdleBehavior DISABLED_IDLE_BEHAVIOR = IdleBehavior.COAST; // Disabled Indexer is set on Coast
        public static final IdleBehavior ENABLED_IDLE_BEHAVIOR = IdleBehavior.BRAKE; // Enabled Indexer is set on Brake

        public static final double FRONT_ROLLERS_REDUCTION = 1.0; // Motor is directly on shaft so reduction is set [1]
        public static final double BACK_ROLLERS_REDUCTION = (1.0 / 2.89) * (16.0 / 30.0); // Back roller Reduction [teeth over teeth * teeth over teeth]

        public static final double FRONT_ROLLERS_DIAMETER = 2.0; // Diameter of Front Rollers [inches]
        public static final double BACK_ROLLERS_DIAMETER = 2.0; // Diameter of Back Rollers [inches]

        public static final int START_BEAM = 9; // Number of port that the starting beam break is in
        public static final int END_BEAM = 1; // Number of port that the end of the beam break is in
        public static final double FEED_WAIT = 1.0; // Wait time for the feed motors to stop after the beam resets [s]

        public static final double FRONT_ROLLERS_P = 0.0;
        public static final double FRONT_ROLLERS_I = 0.0;
        public static final double FRONT_ROLLERS_D = 0.0;

        public static final double BACK_ROLLERS_P = 0.0;
        public static final double BACK_ROLLERS_I = 0.0;
        public static final double BACK_ROLLERS_D = 0.0;

        public static final double FRONT_INDEX_SPEED = 100.0; // Front Roller speed [Inches per s]
        public static final double BACK_INDEX_SPEED = 20.0; // Back Roller speed [Inches per s]
        public static final double FEED_SPEED = 100.0; // Feed Speed [Inches per s]
        public static final double REVERSE_SPEED = 0.0; // Reversing speed

        public static final Config FRONT_SYSID_ROUTINE_CONFIG = new Config(
            Units.Volts.of(1).per(Units.Second), 
            Units.Volts.of(3),
            Units.Seconds.of(10)
        );

        public static final Config BACK_SYSID_ROUTINE_CONFIG = new Config(
            Units.Volts.of(1.25).per(Units.Second), 
            Units.Volts.of(4),
            Units.Seconds.of(10)
        );
    }

    public final class ArmConstants {

        public static final int PIVOT = 35; // Pivot CANBUS ID [CANBUS]
        public static final boolean PIVOT_INVERTED = true; // Arm pivot forward/reverse

        public static final IdleBehavior DISABLED_IDLE_BEHAVIOR = IdleBehavior.COAST; // Sets Arm to coast while disabled
        public static final IdleBehavior ENABLED_IDLE_BEHAVIOR = IdleBehavior.BRAKE;  // Sets Arm to brake while enabled

        public static final int PIVOT_ENCODER = 2; // Pivot Ecoder DIO Port
        public static final double PIVOT_REDUCTION = (1.0 / 36.0) * (36.0 / 44.0) * (15.0 / 54.0); // Pivot Reductio [teeth over teeth]

        public static final double PIVOT_OFFSET = 0.76; // Degree offset to 0 on the pivot [Degree]

        public static final int LOWER_BOUND = 6; // Lower Limit Switch DIO Port
        public static final int UPPER_BOUND = 5; // Upper Limit Switch DIO Port

        public static final double PIVOT_P = 0.15; // Proportional Constant for the Arm [% per Degree]
        public static final double PIVOT_I = 0.15; // Integral Constant for the Arm [% per Degree * 20ms]
        public static final double PIVOT_IZ = 0.25; // Zone where the Integral activates [Degrees]
        public static final double PIVOT_D = 0.0075; // Derivative Constant for the Arm [% per Degree per 20ms]
        public static final double INPUT_TOLERANCE = 0.075; // It is the minimum amount of input to be able to be considered [%]
        public static final double PIVOT_TOLERANCE = 0.5; // This is when the PID Controller says that the Arm's pivot position is good enough [degrees]

        public static final double LOCK_POSITION = 22.0; // How long the Arm is locked in position for [s]
        public static double PIVOT_POSITION = 22.0; // How long the Arm is locked in it's pivot for [s]

        public static final Config SYSID_ROUTINE_CONFIG = new Config(
            Units.Volts.of(0.25).per(Units.Second), 
            Units.Volts.of(3.0),
            Units.Seconds.of(10)
        );
    }

    public final class ShooterConstants {

        public static final int TOP_ROLLERS = 36; // Shooter Top CANBUS ID [CANBUS]
        public static final int BOTTOM_ROLLERS = 37; // Shooter Bottom CANBUS ID [CANBUS]

        public static final boolean TOP_ROLLERS_INVERTED = false; // Top Rollers forward/backward
        public static final boolean BOTTOM_ROLLERS_INVERTED = true; // Bottom Rollers forward/backward

        public static final IdleBehavior DISABLED_IDLE_BEHAVIOR = IdleBehavior.COAST; // When Disabled, the Shooter is in coast mode
        public static final IdleBehavior ENABLED_IDLE_BEHAVIOR = IdleBehavior.COAST; // Whe Enabled, the Shooter is in coast mode

        public static final double TOP_ROLLERS_REDUCTION = 1.0; // Top Rollers reduction [teeth to teeth]
        public static final double BOTTOM_ROLLERS_REDUCTION = 1.0; // Bottom Rollers reductio [teeth to teeth]

        public static final double TOP_ROLLERS_DIAMETER = 4.0; // Diameter of top rollers [inches]
        public static final double BOTTOM_ROLLERS_DIAMETER = 4.0; // Diameter of bottom rollers [inches]

        public static double SHOOT_SPEED = 600.0; // Speed of the shooter [Inches per s]
        public static double SHOOT_TOLERANCE = 25.0; // Uses the Bang Bang Controller to either set the motor power to 100 or kill it (must be coast) [Inches per s]

        public static final Config SYSID_ROUTINE_CONFIG = new Config(
            Units.Volts.of(0.75).per(Units.Second), 
            Units.Volts.of(7),
            Units.Seconds.of(10)
        );
    }

    public enum IdleBehavior {
        COAST,
        BRAKE
    }
}
