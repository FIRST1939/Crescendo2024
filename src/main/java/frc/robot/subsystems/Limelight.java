package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
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
    private Rotation3d latestRotation;
    private double latestTimestamp;

    public Limelight () {

        LimelightHelpers.setLEDMode_PipelineControl("limelight");
        LimelightHelpers.setCropWindow("limelight", -1, 1, -1, 1);
    }

    @Override
    public void periodic () {

        LimelightResults limelightResults = LimelightHelpers.getLatestResults("limelight");
        Results targetingResults = limelightResults.targetingResults;

        double latestTimestamp = targetingResults.timestamp_RIOFPGA_capture;

        if (this.latestTimestamp != latestTimestamp) {

            if (!DriverStation.getAlliance().isPresent()) {

                this.latestPose = targetingResults.getBotPose2d();
                this.latestRotation = targetingResults.getBotPose3d().getRotation();
            } else if (DriverStation.getAlliance().get() == Alliance.Red) { 
                
                this.latestPose = targetingResults.getBotPose2d_wpiRed(); 
                this.latestRotation = targetingResults.getBotPose3d_wpiRed().getRotation();
            } else { 
                
                this.latestPose = targetingResults.getBotPose2d_wpiBlue(); 
                this.latestRotation = targetingResults.getBotPose3d_wpiBlue().getRotation();
            }
        }

        this.latestTimestamp = latestTimestamp;
        double lastUpdated = Timer.getFPGATimestamp() - latestTimestamp;
        SmartDashboard.putString("Limelight Last Updated", (Math.round(lastUpdated * 10) / 10.0) + "s Ago");

        if (lastUpdated >= 20) { Alerts.limelightDetections.set(true); }
        else { Alerts.limelightDetections.set(false); }
    }

    public Pose2d getLatestPose () { return this.latestPose; }
    public Rotation3d getLatestRotation () { return this.latestRotation; }
    public double getLatestTimestamp () { return this.latestTimestamp; }
}
