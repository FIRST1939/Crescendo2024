package frc.robot.util;

public class MotorUtil {

    public static double getMaxVelocity (MotorType motorType, double reduction, double diameter) {

        int maxRPM = 0;

        switch (motorType) {

            case NEO550:
                maxRPM = 11710;
                break;

            case NEO:
                maxRPM = 5820;
                break;

            case FALCON:
                maxRPM = 6380;
                break;

            case VORTEX:
                maxRPM = 6784;
                break;
        }

        return maxRPM * reduction * (Math.PI * diameter) * (1 / 60.0);
    }

    public static enum MotorType { NEO550, NEO, FALCON, VORTEX }
}
