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

        private static final int FREE_SPEED = 5676; // Motor Free Speed [RPM]
        private static final double WHEEL_DIAMETER = 0.10033; // Diameter of the Wheel [m]

        private static final double DRIVE_REDUCTION = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0); // Gear Reduction of the Driving Motors [Wheel Revolution / Motor Revolution]
        private static final double STEER_REDUCTION = (7.0 / 150.0); // Gear Reduction of the Steering Motors [Wheel Revolution / Motor Revolution]

        public static final double MAX_DRIVE_SPEED = FREE_SPEED * DRIVE_REDUCTION * (WHEEL_DIAMETER * Math.PI) / 60.0; // Maximum Driving Speed [m / s]
        public static final double MAX_STEER_SPEED = FREE_SPEED * STEER_REDUCTION * (2 * Math.PI) / 60.0; // Maximum Steering Speed [rad / s]
    
        public static final double TRANSLATION_DEADBAND = 0.0825; // Joystick Deadband [%]
        public static final double OMEGA_DEADBAND = 0.125; // Joystick Deadband [%]

        public static final double TURN_CONSTANT = 0.125; // Turning Power [%]
        public static final double LOCK_TIME = 10.0; // Lock Time While Disabled [s]

        public static final double REPLANNING_TOTAL_ERROR = 0.15; // Total Error to Replan Path [m]
        public static final double REPLANNING_ERROR_SPIKE = 0.1; // Spike in Error to Replan Path [m / 20ms]
    }

    public final class IntakeConstants {

        public static final int TOP_ROLLER = 0;
        public static final int BOTTOM_ROLLER = 0;

        public static final boolean TOP_ROLLER_INVERTED = false;
        public static final boolean BOTTOM_ROLLER_INVERTED = false;

        public static final double TOP_ROLLER_REDUCTION = (15.0 / 30.0);
        public static final double BOTTOM_ROLLER_REDUCTION = (1.0 / 2.89);

        public static final Config TOP_SYSID_ROUTINE_CONFIG = new Config(
            Units.Volts.of(1).per(Units.Second), 
            Units.Volts.of(7),
            Units.Seconds.of(10)
        );

        public static final Config BOTTOM_SYSID_ROUTINE_CONFIG = new Config(
            Units.Volts.of(1).per(Units.Second), 
            Units.Volts.of(7),
            Units.Seconds.of(10)
        );
    }

    public final class IndexerConstants {

        public static final int TOP_ROLLERS = 0;
        public static final int BOTTOM_ROLLERS = 0;

        public static final boolean TOP_ROLLERS_INVERTED = false;
        public static final boolean BOTTOM_ROLLERS_INVERTED = false;

        public static final double TOP_ROLLERS_REDUCTION = (1.0 / 2.89) * (16.0 / 30.0);
        public static final double BOTTOM_ROLLERS_REDUCTION = (1.0 / 2.89) * (16.0 / 30.0);

        public static final Config SYSID_ROUTINE_CONFIG = new Config(
            Units.Volts.of(1).per(Units.Second), 
            Units.Volts.of(7),
            Units.Seconds.of(10)
        );
    }

    public final class ArmConstants {

        public static final int PIVOT = 0;
        public static final boolean PIVOT_INVERTED = false;

        public static final int PIVOT_ENCODER = 0;
        public static final double PIVOT_REDUCTION = (1.0 / 36.0) * (36.0 / 44.0) * (15.0 / 54.0);

        public static final Config SYSID_ROUTINE_CONFIG = new Config(
            Units.Volts.of(1).per(Units.Second), 
            Units.Volts.of(7),
            Units.Seconds.of(10)
        );
    }

    public final class ShooterConstants {

        public static final int TOP_ROLLERS = 0;
        public static final int BOTTOM_ROLLERS = 0;

        public static final boolean TOP_ROLLERS_INVERTED = false;
        public static final boolean BOTTOM_ROLLERS_INVERTED = false;

        public static final double TOP_ROLLERS_REDUCTION = 1.0;
        public static final double BOTTOM_ROLLERS_REDUCTION = 1.0;

        public static final Config SYSID_ROUTINE_CONFIG = new Config(
            Units.Volts.of(1).per(Units.Second), 
            Units.Volts.of(7),
            Units.Seconds.of(10)
        );
    }
}
