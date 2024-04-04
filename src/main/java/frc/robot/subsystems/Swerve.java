package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;

import com.pathplanner.lib.commands.PathfindHolonomic;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;
import frc.robot.util.Constants.IdleBehavior;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.parser.PIDFConfig;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Swerve extends SubsystemBase {
    
    private final SwerveDrive swerveDrive;
    private Field2d field = new Field2d();

    public Swerve () throws IOException {

        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.NONE;
        File swerveConfigurationDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
        this.swerveDrive = new SwerveParser(swerveConfigurationDirectory).createSwerveDrive(Constants.SwerveConstants.MAX_DRIVE_SPEED);
        
        for (SwerveModule swerveModule : this.swerveDrive.getModules()) {

            swerveModule.getAngleMotor().configurePIDWrapping(-180, 180);
        }

        this.configureSwerveDashboard();
    }

    @Override
    public void periodic () { 
        
        this.field.setRobotPose(this.getPose());
        SmartDashboard.putNumber("Distance", this.getSpeakerDistance()); 
    }
    public PIDFConfig getHeadingPIDFConfig () { return this.swerveDrive.swerveController.config.headingPIDF; }
    public SwerveDriveConfiguration getConfiguration () { return this.swerveDrive.swerveDriveConfiguration; }

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
                translation.getX(), translation.getY(), rotation.getRadians(),
                this.swerveDrive.getOdometryHeading().getRadians(),
                this.swerveDrive.getMaximumVelocity()
            )
        );
    }

    public Command pathfind (Pose2d target) {

        return new PathfindHolonomic(
            target,
            new PathConstraints(5.05, 4.0, 180.0, 180.0),
            this::getPose,
            this::getRobotVelocity,
            this::setChassisSpeeds,
            new HolonomicPathFollowerConfig(
                new PIDConstants(5.0, 0.0, 0.0),
                new PIDConstants(
                    this.getHeadingPIDFConfig().p,
                    this.getHeadingPIDFConfig().i,
                    this.getHeadingPIDFConfig().d
                ),
                Constants.SwerveConstants.MAX_DRIVE_SPEED,
                this.getConfiguration().getDriveBaseRadiusMeters(),
                new ReplanningConfig()
            ),
            this
        );
    }

    public SwerveDrivePoseEstimator getPoseEstimator () { return this.swerveDrive.swerveDrivePoseEstimator; }
    public Rotation2d getHeading () { return this.swerveDrive.getYaw(); }
    public Pose2d getPose () { return this.swerveDrive.getPose(); }

    public double getSpeakerDistance () {

        Translation2d speaker;
        if (DriverStation.getAlliance().get() == Alliance.Blue) { speaker = Constants.SwerveConstants.BLUE_SPEAKER; }
        else { speaker = Constants.SwerveConstants.RED_SPEAKER; }
        
        return this.getPose().getTranslation().getDistance(speaker);
    }

    public ChassisSpeeds getRobotVelocity () { return this.swerveDrive.getRobotVelocity(); }
    public ChassisSpeeds getFieldVelocity () { return this.swerveDrive.getFieldVelocity(); }

    public void zeroGyro () {

        Translation2d translation = this.getPose().getTranslation();
        Rotation2d rotation = new Rotation2d();

        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {

            Rotation2d allianceOriented = Rotation2d.fromDegrees(180);
            rotation = rotation.plus(allianceOriented); 
        }

        this.resetOdometry(new Pose2d(translation, rotation));
    }

    public void resetOdometry (Pose2d pose) { this.swerveDrive.resetOdometry(pose); }
    public void setChassisSpeeds (ChassisSpeeds chassisSpeeds) { this.swerveDrive.drive(chassisSpeeds); }

    public void addVisionMeasurement (Pose2d limelightPose, double timestamp, Matrix<N3,  N1> standardDeviations) { 
        
        Translation2d translation = limelightPose.getTranslation();
        Pose2d pose = new Pose2d(translation, this.swerveDrive.getOdometryHeading());

        this.swerveDrive.addVisionMeasurement(pose, timestamp, standardDeviations); 
    }

    public void setIdleBehavior (IdleBehavior idleBehavior) {

        if (idleBehavior == IdleBehavior.BRAKE) { this.swerveDrive.setMotorIdleMode(true); }
        else { this.swerveDrive.setMotorIdleMode(false); }
    }

    public void lock () { this.swerveDrive.lockPose(); }
}
