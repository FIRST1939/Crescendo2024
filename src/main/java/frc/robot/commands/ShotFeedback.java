package frc.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Logging;

public class ShotFeedback extends Command {
    
    private Logging logging;

    private boolean shotMade = false;
    private boolean shotStraight = true;
    private String shotInfo = "";

    private SendableChooser<String> shotAngle;

    public ShotFeedback (Logging logging) {

        this.logging = logging;
        this.addRequirements(this.logging);
    }

    @Override
    public void initialize () {

        SmartDashboard.putBoolean("Shot Made", this.shotMade);
        SmartDashboard.putBoolean("Shot Straight", this.shotStraight);
        SmartDashboard.putString("Shot Info", this.shotInfo);
        SmartDashboard.putBoolean("Submit Shot", false);

        this.shotAngle = new SendableChooser<>();
        this.shotAngle.setDefaultOption("Center", "Center");
        this.shotAngle.addOption("Low", "Low");
        this.shotAngle.addOption("High", "High");
        this.shotAngle.addOption("N/A", "N/A");

        Shuffleboard.getTab("Experimentation").add("Shot Angle", this.shotAngle);
    }

    @Override
    public boolean isFinished () { return SmartDashboard.getBoolean("Submit Shot", false); }

    @Override
    public void end (boolean interrupted) {

        this.shotMade = SmartDashboard.getBoolean("Shot Made", false);
        this.shotStraight = SmartDashboard.getBoolean("Shot Straight", true);
        this.shotInfo = SmartDashboard.getString("Shot Info", "");

        this.logging.logShot(this.shotMade, this.shotStraight, this.shotInfo, this.shotAngle.getSelected());
    }
}
