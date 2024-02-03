package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Swerve extends SubsystemBase {
    
    private final SwerveDrive swerveDrive;
    private SendableChooser<Command> autonomousChooser;
    private Field2d field = new Field2d();

    public Swerve () throws IOException {

        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.NONE;
        File swerveConfigurationDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
        this.swerveDrive = new SwerveParser(swerveConfigurationDirectory).createSwerveDrive(Constants.SwerveConstants.MAX_DRIVE_SPEED);
        
        for (SwerveModule swerveModule : this.swerveDrive.getModules()) {

            swerveModule.getAngleMotor().configurePIDWrapping(-180, 180);
        }

        this.initializePathPlanner();
        this.configureSwerveDashboard();
    }

    @Override
    public void periodic () { this.field.setRobotPose(this.getPose()); }
    public SwerveDriveConfiguration getConfiguration () { return this.swerveDrive.swerveDriveConfiguration; }

    private void initializePathPlanner () {

        AutoBuilder.configureHolonomic(
            this::getPose, this::resetOdometry,
            this::getRobotVelocity, this::setChassisSpeeds,
            new HolonomicPathFollowerConfig(
                new PIDConstants(5.0, 0.0, 0.0),
                new PIDConstants(
                    this.swerveDrive.swerveController.config.headingPIDF.p,
                    this.swerveDrive.swerveController.config.headingPIDF.i,
                    this.swerveDrive.swerveController.config.headingPIDF.d
                ),
                Constants.SwerveConstants.MAX_DRIVE_SPEED,
                this.getConfiguration().getDriveBaseRadiusMeters(),
                new ReplanningConfig(
                    true, true, 
                    Constants.SwerveConstants.REPLANNING_TOTAL_ERROR,
                    Constants.SwerveConstants.REPLANNING_ERROR_SPIKE
                )
            ),
            () -> {

                var alliance = DriverStation.getAlliance();
                return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
            },
            this
        );

        this.autonomousChooser = AutoBuilder.buildAutoChooser();
        Shuffleboard.getTab("Autonomous").add("Autonomous Chooser", this.autonomousChooser);
    }

    public void configureSwerveDashboard () {

        SmartDashboard.putData("Swerve", new Sendable() {
            
            @Override
            public void initSendable (SendableBuilder sendableBuilder) {

                sendableBuilder.setSmartDashboardType("SwerveDrive");

                sendableBuilder.addDoubleProperty("Front Left Angle", () -> swerveDrive.getModules()[0].getState().angle.getDegrees(), null);
                sendableBuilder.addDoubleProperty("Front Left Velocity", () -> swerveDrive.getModules()[0].getState().speedMetersPerSecond, null);

                sendableBuilder.addDoubleProperty("Front Right Angle", () -> swerveDrive.getModules()[1].getState().angle.getDegrees(), null);
                sendableBuilder.addDoubleProperty("Front Right Velocity", () -> swerveDrive.getModules()[1].getState().speedMetersPerSecond, null);

                sendableBuilder.addDoubleProperty("Back Left Angle", () -> swerveDrive.getModules()[2].getState().angle.getDegrees(), null);
                sendableBuilder.addDoubleProperty("Back Left Velocity", () -> swerveDrive.getModules()[2].getState().speedMetersPerSecond, null);

                sendableBuilder.addDoubleProperty("Back Right Angle", () -> swerveDrive.getModules()[3].getState().angle.getDegrees(), null);
                sendableBuilder.addDoubleProperty("Back Right Velocity", () -> swerveDrive.getModules()[3].getState().speedMetersPerSecond, null);

                sendableBuilder.addDoubleProperty("Robot Angle", () -> getHeading().getDegrees(), null);
            }
        });

        Shuffleboard.getTab("Autonomous").add("Field", this.field);
        PathPlannerLogging.setLogActivePathCallback((poses) -> { this.field.getObject("trajectory").setPoses(poses); });
    }

    public void driveAngularVelocity (Translation2d translation, double omega) {

        this.swerveDrive.setHeadingCorrection(false);
        this.swerveDrive.drive(translation, omega, true, false);
    }

    public void driveHeadingLock (Translation2d translation, Rotation2d rotation) {

        this.swerveDrive.setHeadingCorrection(true);

        this.swerveDrive.driveFieldOriented(
            this.swerveDrive.getSwerveController().getTargetSpeeds(
                translation.getX(), translation.getY(),
                rotation.getCos(), rotation.getSin(),
                this.swerveDrive.getOdometryHeading().getRadians(),
                this.swerveDrive.getMaximumVelocity()
            )
        );
    }

    public Rotation2d getHeading () { return this.swerveDrive.getYaw(); }
    public Pose2d getPose () { return this.swerveDrive.getPose(); }

    public ChassisSpeeds getRobotVelocity () { return this.swerveDrive.getRobotVelocity(); }
    public ChassisSpeeds getFieldVelocity () { return this.swerveDrive.getFieldVelocity(); }

    public void zeroGyro () { this.swerveDrive.zeroGyro(); }
    public void resetOdometry (Pose2d pose) { this.swerveDrive.resetOdometry(pose); }
    public void setChassisSpeeds (ChassisSpeeds chassisSpeeds) { this.swerveDrive.drive(chassisSpeeds); }

    public void addVisionMeasurement (Pose2d pose2d, double timestamp, Rotation3d rotation3d) { 

        this.swerveDrive.addVisionMeasurement(pose2d, timestamp);
        this.swerveDrive.setGyroOffset(rotation3d);
    }

    public void lock () { this.swerveDrive.lockPose(); }
    public void setBrakeMode (boolean brake) { this.swerveDrive.setMotorIdleMode(brake); }
    public Command getAutonomousCommand () { return this.autonomousChooser.getSelected(); }
}
