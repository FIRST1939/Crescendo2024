package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;

import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.Constants;
import frc.robot.util.Constants.IdleBehavior;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
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

        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        File swerveConfigurationDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
        this.swerveDrive = new SwerveParser(swerveConfigurationDirectory).createSwerveDrive(Constants.SwerveConstants.MAX_DRIVE_SPEED);
        
        for (SwerveModule swerveModule : this.swerveDrive.getModules()) {

            swerveModule.getAngleMotor().configurePIDWrapping(-180, 180);
        }

        this.configureSwerveDashboard();
    }

    @Override
    public void periodic () { this.field.setRobotPose(this.getPose()); }
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
    public void addVisionMeasurement (Pose2d pose2d, double timestamp) { this.swerveDrive.addVisionMeasurement(pose2d, timestamp); }

    public void setIdleBehavior (IdleBehavior idleBehavior) {

        if (idleBehavior == IdleBehavior.BRAKE) { this.swerveDrive.setMotorIdleMode(true); }
        else { this.swerveDrive.setMotorIdleMode(false); }
    }

    public void lock () { this.swerveDrive.lockPose(); }
    
    public Command getDriveSysidRoutine () {

        SysIdRoutine sysIdRoutine = SwerveDriveTest.setDriveSysIdRoutine(
            Constants.SwerveConstants.DRIVE_SYSID_CONFIG,
            this, this.swerveDrive,
            12.0
        );

        return SwerveDriveTest.generateSysIdCommand(
            sysIdRoutine, 
            Constants.SwerveConstants.DRIVE_SYSID_CONFIG.m_timeout.magnitude(), 
            Constants.SwerveConstants.DRIVE_SYSID_QUASISTATIC_TIMEOUT, 
            Constants.SwerveConstants.DRIVE_SYSID_DYNAMIC_TIMEOUT
        );
    }

    public Command getAngleSysidRoutine () {

        SysIdRoutine sysIdRoutine = SwerveDriveTest.setAngleSysIdRoutine(
            Constants.SwerveConstants.ANGLE_SYSID_CONFIG,
            this, this.swerveDrive
        );

        return SwerveDriveTest.generateSysIdCommand(
            sysIdRoutine, 
            Constants.SwerveConstants.ANGLE_SYSID_CONFIG.m_timeout.magnitude(), 
            Constants.SwerveConstants.ANGLE_SYSID_QUASISTATIC_TIMEOUT, 
            Constants.SwerveConstants.ANGLE_SYSID_DYNAMIC_TIMEOUT
        );
    }
}
