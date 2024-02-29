package frc.robot.util;

import frc.lib.Alert;
import frc.lib.Alert.AlertType;

public final class Alerts {
    
    public static final Alert versionControl = new Alert("Current Code Not Under Version Control", AlertType.WARNING);

    public static final Alert limelightDetections = new Alert("Limelight Hasn't Detected AprilTag in >20s", AlertType.WARNING);
    public static final Alert driverOneDisconnected = new Alert("Driver One Controller Disconnected", AlertType.ERROR);
    public static final Alert driverTwoDisconnected = new Alert("Driver Two Controller Disconnected", AlertType.ERROR);

    public static final Alert intakeStateMachine = new Alert("Intake State Machine Offline", AlertType.ERROR);
    public static final Alert indexerStateMachine = new Alert("Indexer State Machine Offline", AlertType.ERROR);
    public static final Alert armStateMachine = new Alert("Arm State Machine Offline", AlertType.ERROR);
    public static final Alert shooterStateMachine = new Alert("Shooter State Machine Offline", AlertType.ERROR);
}
