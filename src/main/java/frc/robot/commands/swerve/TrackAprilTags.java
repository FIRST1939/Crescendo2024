package frc.robot.commands.swerve;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class TrackAprilTags extends Command {
    
    private final Swerve swerve;
    private final Limelight limelight;

    public TrackAprilTags (Swerve swerve, Limelight limelight) {

        this.swerve = swerve;
        this.limelight = limelight;
        this.addRequirements(this.limelight);
    }

    @Override
    public void execute () {

        if (DriverStation.isAutonomous()) { return; }
        if (!this.limelight.areValidMeasurements()) { return; }

        Translation2d estimatedTranslation = this.swerve.getPoseEstimator().getEstimatedPosition().getTranslation();
        double poseDifference = estimatedTranslation.getDistance(this.limelight.getLatestPose().getTranslation());

        double xyStandardDeviation = 0.0;

        if (this.limelight.getLatestTargets() >= 2) {

            // Multiple Targets Detected
            xyStandardDeviation = 0.5;
        } else if (this.limelight.getLatestTargetArea() > 0.8 && poseDifference < 0.5) {

            // 1 Target with Large Area and Close to Estimated Pose
            xyStandardDeviation = 1.0;
        } else if (this.limelight.getLatestTargetArea() > 0.1 && poseDifference < 0.3) {

            // 1 Target Farther Away but Close to Estimated Pose
            xyStandardDeviation = 2.0;
        } else { 
            
            // Insufficient Targeting Data
            return; 
        }

        Matrix<N3, N1> standardDeviations = VecBuilder.fill(
            xyStandardDeviation, xyStandardDeviation, 
            Double.MAX_VALUE
        );

        this.swerve.addVisionMeasurement(
            this.limelight.getLatestPose(), 
            Timer.getFPGATimestamp() - (this.limelight.getLatestDelay() / 1000.0),
            standardDeviations
        );
    }

    @Override
    public boolean isFinished () { return false; }
}