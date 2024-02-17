package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.LimelightHelpers;
import frc.lib.LimelightHelpers.LimelightResults;
import frc.lib.LimelightHelpers.Results;
import frc.robot.util.Alerts;

public class Limelight extends SubsystemBase {
    
    private Pose2d latestPose;
    private double latestDelay;
    
    private boolean validMeasurements;
    private Timer usageTimer = new Timer();

    public Limelight () {

        LimelightHelpers.setPipelineIndex("limelight", 0);
        LimelightHelpers.setLEDMode_PipelineControl("limelight");

        LimelightHelpers.setCropWindow("limelight", -1, 1, -1, 1);

        this.usageTimer.reset();
        this.usageTimer.start();
    }

    @Override
    public void periodic () {

        LimelightResults limelightResults = LimelightHelpers.getLatestResults("limelight");
        Results targetingResults = limelightResults.targetingResults;
        this.validMeasurements = targetingResults.valid;

        if (this.validMeasurements) {

            if (!DriverStation.getAlliance().isPresent()) { this.latestPose = targetingResults.getBotPose2d(); } 
            else if (DriverStation.getAlliance().get() == Alliance.Red) { this.latestPose = targetingResults.getBotPose2d_wpiRed(); } 
            else { this.latestPose = targetingResults.getBotPose2d_wpiBlue(); }

            this.latestDelay = targetingResults.latency_capture + targetingResults.latency_pipeline;
            this.usageTimer.restart();
        }

        double lastUpdated = this.usageTimer.get();
        SmartDashboard.putString("Limelight Last Updated", (Math.round(lastUpdated * 10) / 10.0) + "s Ago");

        if (lastUpdated >= 20) { Alerts.limelightDetections.set(true); }
        else { Alerts.limelightDetections.set(false); }
    }

    public Pose2d getLatestPose () { return this.latestPose; }
    public double getLatestDelay () { return this.latestDelay; }
    public boolean areValidMeasurements () { return this.validMeasurements; }
}
