package frc.robot.subsystems;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;

public class Logging extends SubsystemBase {
    
    private DoubleSupplier armAngle;
    private DoubleSupplier topShooterSpeed;
    private DoubleSupplier bottomShooterSpeed;
    private BooleanSupplier indexerBeamBreak;

    private boolean currentlyLogging = false;
    private ArrayList<LogEntry> currentLogEntries;
    private ArrayList<Run> runs;

    public Logging (DoubleSupplier armAngle, DoubleSupplier topShooterSpeed, DoubleSupplier bottomShooterSpeed, BooleanSupplier indexerBeamBreak) {

        this.armAngle = armAngle;
        this.topShooterSpeed = topShooterSpeed;
        this.bottomShooterSpeed = bottomShooterSpeed;
        this.indexerBeamBreak = indexerBeamBreak;
        this.currentLogEntries = new ArrayList<>();
        this.runs = new ArrayList<>();
    }

    @Override
    public void periodic () {

        if (this.currentlyLogging) { 

            LogEntry logEntry = new LogEntry(this.armAngle.getAsDouble(), this.topShooterSpeed.getAsDouble(), this.bottomShooterSpeed.getAsDouble(), this.indexerBeamBreak.getAsBoolean());
            this.currentLogEntries.add(logEntry);
        }
    }

    public void setLogging (boolean currentlyLogging) { this.currentlyLogging = currentlyLogging; }

    public void logShot (boolean shotMade, boolean shotStraight, String shotInfo, String shotAngle) {

        Run run = new Run(
            this.currentLogEntries,
            shotMade, shotStraight,
            shotInfo, shotAngle
        );

        this.runs.add(run);
        this.currentLogEntries = new ArrayList<>();
    }

    public void saveLog () {

        JSONObject log = new JSONObject();
        JSONObject constants = new JSONObject();
        JSONArray runs = new JSONArray();

        constants.put("indexer_speed", Constants.IndexerConstants.FEED_SPEED);
        constants.put("arm_angle", Constants.ArmConstants.PIVOT_POSITION);
        constants.put("top_shooter_speed", Constants.ShooterConstants.TOP_SHOOT_SPEED);
        constants.put("bottom_shooter_speed", Constants.ShooterConstants.BOTTOM_SHOOT_SPEED);

        for (Run run : this.runs) {

            JSONObject runObject = new JSONObject();
            JSONArray logEntries = new JSONArray();

            for (LogEntry logEntry : run.logEntries) {

                JSONObject logEntryObject = new JSONObject();
                logEntryObject.put("timestamp", logEntry.timestamp);
                logEntryObject.put("arm_angle", logEntry.armAngle);
                logEntryObject.put("top_shooter_speed", logEntry.topShooterSpeed);
                logEntryObject.put("bottom_shooter_speed", logEntry.bottomShooterSpeed);
                logEntryObject.put("indexer_beam_break", logEntry.indexerBeamBreak);

                logEntries.add(logEntryObject);
            }

            runObject.put("log_entries", logEntries);
            runObject.put("shot_made", run.shotMade);
            runObject.put("shot_straight", run.shotStraight);
            runObject.put("shot_info", run.shotInfo);
            runObject.put("shot_angle", run.shotAngle);

            runs.add(runObject);
        }

        log.put("constants", constants);
        log.put("runs", runs);

        try {

            double timestamp = Timer.getFPGATimestamp();
            File file = new File("/U/logs/" + timestamp + ".json");
            if (!file.exists()) { file.createNewFile(); }

            FileWriter fileWriter = new FileWriter("/U/logs/" + timestamp + ".json");
            fileWriter.write(log.toJSONString());
            fileWriter.close();
        } catch (IOException ioException) {

            ioException.printStackTrace();
        }
    }
    
    class LogEntry {

        public final double timestamp;
        public final double armAngle;
        public final double topShooterSpeed;
        public final double bottomShooterSpeed;
        public final boolean indexerBeamBreak;

        public LogEntry (double armAngle, double topShooterSpeed, double bottomShooterSpeed, boolean indexerBeamBreak) {

            this.timestamp = Timer.getFPGATimestamp();
            
            this.armAngle = armAngle;
            this.topShooterSpeed = topShooterSpeed;
            this.bottomShooterSpeed = bottomShooterSpeed;
            this.indexerBeamBreak = indexerBeamBreak;
        }
    }

    class Run {

        public final ArrayList<LogEntry> logEntries;
        public final boolean shotMade;
        public final boolean shotStraight;
        public final String shotInfo;
        public final String shotAngle;

        public Run (ArrayList<LogEntry> logEntries, boolean shotMade, boolean shotStraight, String shotInfo, String shotAngle) {

            this.logEntries = logEntries;
            this.shotMade = shotMade;
            this.shotStraight = shotStraight;
            this.shotInfo = shotInfo;
            this.shotAngle = shotAngle;
        }
    }
}
