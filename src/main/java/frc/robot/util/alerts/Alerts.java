package frc.robot.util.alerts;

import frc.robot.util.alerts.Alert.AlertType;

public final class Alerts {
    
    public static final class SwerveAlerts {

        public static final Alert swerveInitialized = new Alert("Swerve Failed to Initialize", AlertType.ERROR);
    }
}
