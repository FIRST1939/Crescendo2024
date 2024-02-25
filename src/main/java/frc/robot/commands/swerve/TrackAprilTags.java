package frc.robot.commands.swerve;

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

        if (!this.limelight.areValidMeasurements()) { return; }

        this.swerve.addVisionMeasurement(
            this.limelight.getLatestPose(), 
            Timer.getFPGATimestamp() - (this.limelight.getLatestDelay() / 1000.0)
        );
    }

    @Override
    public boolean isFinished () { return false; }
}
