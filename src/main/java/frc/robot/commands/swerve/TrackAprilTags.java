package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class TrackAprilTags extends Command {
    
    private final Swerve swerve;
    private final Limelight limelight;
    private double latestTimestamp;

    public TrackAprilTags (Swerve swerve, Limelight limelight) {

        this.swerve = swerve;
        this.limelight = limelight;
        this.addRequirements(this.limelight);
    }

    @Override
    public void execute () {

        if (this.limelight.getLatestTimestamp() == this.latestTimestamp) { return; }

        this.swerve.addVisionMeasurement(
            this.limelight.getLatestPose(), 
            this.limelight.getLatestTimestamp(), 
            this.limelight.getLatestRotation()
        );

        this.latestTimestamp = this.limelight.getLatestTimestamp();
    }

    @Override
    public boolean isFinished () { return false; }
}
