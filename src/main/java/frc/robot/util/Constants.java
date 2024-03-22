package frc.robot.util;

import java.util.List;

import edu.wpi.first.math.geometry.Translation3d;
import swervelib.math.Matter;

public final class Constants {
    
    public final class RobotConstants {

        public static final double ROBOT_MASS = 60.3; // Robot Weight [kg]
        public static final double LOOP_TIME = 0.13; // Control Loop Tick Timing [s]
        public static final double MINIMUM_BATTERY_VOLTAGE = 12.2; // Minimum Battery Voltage [V]
        public static final List<Matter> WEIGHT_DISTRIBUTION = List.of(new Matter(new Translation3d(0.0103, -0.0064, 0.1890), 53.9953));

        public static final int BLINKIN = 1;
    }

    public final class SwerveConstants {

        public static final IdleBehavior DISABLED_IDLE_BEHAVIOR = IdleBehavior.COAST;
        public static final IdleBehavior ENABLED_IDLE_BEHAVIOR = IdleBehavior.BRAKE;

        private static final int FREE_SPEED = 5820; // Motor Free Speed [RPM]
        private static final double WHEEL_DIAMETER = 0.1016; // Diameter of the Wheel [m]

        private static final double DRIVE_REDUCTION = (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0); // Gear Reduction of the Driving Motors [Wheel Revolution / Motor Revolution]
        private static final double STEER_REDUCTION = (7.0 / 150.0); // Gear Reduction of the Steering Motors [Wheel Revolution / Motor Revolution]

        public static final double MAX_DRIVE_SPEED = FREE_SPEED * DRIVE_REDUCTION * (WHEEL_DIAMETER * Math.PI) / 60.0; // Maximum Driving Speed [m / s]
        public static final double MAX_STEER_SPEED = FREE_SPEED * STEER_REDUCTION * (2 * Math.PI) / 60.0; // Maximum Steering Speed [rad / s]
    
        public static final double TRANSLATION_DEADBAND = 0.0825; // Joystick Deadband [%]
        public static final double OMEGA_DEADBAND = 0.125; // Joystick Deadband [%]

        public static final double TURN_CONSTANT = 0.225; // Turning Power [%]
        public static final double LOCK_TIME = 10.0; // Lock Time While Disabled [s]

        public static final double REPLANNING_TOTAL_ERROR = 0.35; // Total Error to Replan Path [m]
        public static final double REPLANNING_ERROR_SPIKE = 0.25; // Spike in Error to Replan Path [m / 20ms]
    }

    public final class IntakeConstants {

        public static final int TOP_ROLLER = 31;
        public static final int BOTTOM_ROLLER = 32;

        public static final boolean TOP_ROLLER_INVERTED = false;
        public static final boolean BOTTOM_ROLLER_INVERTED = false;

        public static final IdleBehavior DISABLED_IDLE_BEHAVIOR = IdleBehavior.COAST;
        public static final IdleBehavior ENABLED_IDLE_BEHAVIOR = IdleBehavior.BRAKE;

        public static final double TOP_ROLLER_REDUCTION = (15.0 / 30.0);
        public static final double BOTTOM_ROLLER_REDUCTION = (1.0 / 2.89);

        public static final double TOP_ROLLER_DIAMETER = 2.0;
        public static final double BOTTOM_ROLLER_DIAMETER = 1.625;

        public static final double INTAKE_SPEED = 150.0;
        public static final double OUTAKE_SPEED = -100.0;
        public static final double EJECT_SPEED = 240.0;
    }

    public final class ElevatorConstants {

        public static final int LEAD_RAISE = 33;
        public static final int FOLLOWER_RAISE = 34;

        public static final boolean LEAD_RAISE_INVERTED = false;
        public static final boolean FOLLOWER_RAISE_INVERTED = true;

        public static final IdleBehavior DISABLED_IDLE_BEHAVIOR = IdleBehavior.COAST;
        public static final IdleBehavior ENABLED_IDLE_BEHAVIOR = IdleBehavior.BRAKE;

        public static final int RAISE_ENCODER = 8;
        public static final double RAISE_OFFSET = 0.48445;

        public static final int LOWER_BOUND = 3;
        public static final int UPPER_BOUND = 2;

        public static final double RAISE_KS = 0.16;
        public static final double RAISE_KP = 60.0;
        public static final double RAISE_CAP = 8.0;
        public static final double RAISE_TOLERANCE = 0.001;

        public static final double LOCK_POSITION = 0.0;
        public static final double RAISE_POSITION = 2.04;

        public static final double MANUAL_LOWER_SPEED = -0.40;
        public static final double MANUAL_RAISE_SPEED = 0.40;
    }

    public final class IndexerConstants {

        public static final int FRONT_ROLLERS = 35;
        public static final int BACK_ROLLERS = 37;

        public static final boolean FRONT_ROLLERS_INVERTED = true;
        public static final boolean BACK_ROLLERS_INVERTED = true;

        public static final IdleBehavior DISABLED_IDLE_BEHAVIOR = IdleBehavior.COAST;
        public static final IdleBehavior ENABLED_IDLE_BEHAVIOR = IdleBehavior.BRAKE;

        public static final double FRONT_ROLLERS_REDUCTION = 1.0;
        public static final double BACK_ROLLERS_REDUCTION = (44.0 / 42.0);

        public static final double FRONT_ROLLERS_DIAMETER = 2.0;
        public static final double BACK_ROLLERS_DIAMETER = 2.0;

        public static final int START_BEAM = 7;
        public static final int END_BEAM = 6;

        public static final double OVERLOAD_TIME = 0.75;
        public static final double LOAD_TIME = 0.2;
        public static final double FEED_WAIT = 1.0;

        public static final double FRONT_INDEX_SPEED = 200.0;
        public static final double BACK_INDEX_SPEED = 120.0;
        public static final double LOAD_SPEED = -60.0;
        public static final double FEED_SPEED = 700.0;
        public static final double REVERSE_SPEED = -100.0;
        public static final double DROP_SPEED = -560.0;
        public static final double FRONT_EJECT_SPEED = 560.0;
        public static final double BACK_EJECT_SPEED = 510.0;
    }

    public final class ArmConstants {

        public static final int PIVOT = 36;
        public static final boolean PIVOT_INVERTED = true;

        public static final IdleBehavior DISABLED_IDLE_BEHAVIOR = IdleBehavior.COAST;
        public static final IdleBehavior ENABLED_IDLE_BEHAVIOR = IdleBehavior.BRAKE;

        public static final int PIVOT_ENCODER = 1;
        public static final double PIVOT_OFFSET = 0.3514;

        public static final int LOWER_BOUND = 4;
        public static final int UPPER_BOUND = 5;

        public static final double PIVOT_KS = 0.15;
        public static final double PIVOT_KP = 0.2;
        public static final double PIVOT_TOLERANCE = 0.35;

        public static final double LOCK_POSITION = 22.0;
        public static double PIVOT_POSITION = 55.0;
    }

    public final class ShooterConstants {

        public static final int TOP_ROLLERS = 38;
        public static final int BOTTOM_ROLLERS = 39;

        public static final boolean TOP_ROLLERS_INVERTED = false;
        public static final boolean BOTTOM_ROLLERS_INVERTED = true;

        public static final IdleBehavior DISABLED_IDLE_BEHAVIOR = IdleBehavior.COAST;
        public static final IdleBehavior ENABLED_IDLE_BEHAVIOR = IdleBehavior.COAST;

        public static final double TOP_ROLLERS_REDUCTION = 1.0;
        public static final double BOTTOM_ROLLERS_REDUCTION = 1.0;

        public static final double TOP_ROLLERS_DIAMETER = 4.0;
        public static final double BOTTOM_ROLLERS_DIAMETER = 4.0;

        public static final double SPEED_KS = 0.12;
        public static final double SPEED_KV = 0.0085;
        public static final double SPEED_KP = 0.005;
        public static final double SPEED_TOLERANCE = 25.0;

        public static double TOP_SHOOT_SPEED = 800.0;
        public static double BOTTOM_SHOOT_SPEED = 800.0;
        public static final double EJECT_SPEED = 600.0;
    }

    public enum IdleBehavior {
        COAST,
        BRAKE
    }
}
