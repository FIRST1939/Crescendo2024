package frc.robot.commands.swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Constants;

public class Drive extends Command {
    
    private final Swerve swerve;
    private final DoubleSupplier vx, vy;
    private final DoubleSupplier omega;

    private boolean ferryHeadingLock = false;
    private boolean speakerHeadingLock = false;
    private BooleanSupplier leftBumper, rightBumper;

    public Drive (Swerve swerve, DoubleSupplier vx, DoubleSupplier vy, DoubleSupplier omega, BooleanSupplier leftBumper, BooleanSupplier rightBumper) {

        this.swerve = swerve;

        this.vx = vx;
        this.vy = vy;

        this.omega = omega;
        this.leftBumper = leftBumper;
        this.rightBumper = rightBumper;

        this.addRequirements(this.swerve);
    }

    @Override
    public void execute () {

        double vx = this.vx.getAsDouble();
        double vy = this.vy.getAsDouble();

        double rx, ry;

        if (Math.abs(vx) == Math.abs(vy)) { 
            
            rx = Math.signum(vx);
            ry = Math.signum(vy);
        }
        else if (Math.abs(vx) > Math.abs(vy)) {

            rx = Math.signum(vx);
            ry = Math.signum(vy) * Math.abs(vy / vx);
        } else {

            rx = Math.signum(vx) * Math.abs(vx / vy);
            ry = Math.signum(vy);
        }

        double power = Math.sqrt(Math.pow(vx, 2) + Math.pow(vy, 2));
        vx = rx * power;
        vy = ry * power;

        Translation2d translation = new Translation2d(vx, vy);
        
        /**
        translation = SwerveMath.limitVelocity(
            translation, this.swerve.getFieldVelocity(), this.swerve.getPose(), 
            Constants.RobotConstants.LOOP_TIME, Constants.RobotConstants.ROBOT_MASS, 
            Constants.RobotConstants.WEIGHT_DISTRIBUTION, this.swerve.getConfiguration()
        );
        */

        if (this.leftBumper.getAsBoolean()) { this.ferryHeadingLock = true; }
        if (this.rightBumper.getAsBoolean()) { this.speakerHeadingLock = true; }

        if (this.omega.getAsDouble() != 0.0) {

            this.ferryHeadingLock = false;
            this.speakerHeadingLock = false;
        }

        if (this.ferryHeadingLock) {

            Rotation2d corner = new Rotation2d();
            Pose2d pose = this.swerve.getPose();
                
            if (DriverStation.getAlliance().isPresent()) {

                if (DriverStation.getAlliance().get() == Alliance.Blue) { 
                        
                    double headingX = -pose.getX();
                    double headingY = 6.9788 - pose.getY();
                    corner = Rotation2d.fromRadians(Math.atan(headingY / headingX));
                } else {

                    double headingX = 16.5418 - pose.getX();
                    double headingY = 6.9788 - pose.getY();
                    corner = Rotation2d.fromRadians(Math.PI + Math.atan(headingY / headingX));
                }
            }

            this.swerve.driveHeadingLock(translation, corner);
            return;
        }

        if (this.speakerHeadingLock) {

            Rotation2d speaker = new Rotation2d();
            Pose2d pose = this.swerve.getPose();
                
            if (DriverStation.getAlliance().isPresent()) {

                if (DriverStation.getAlliance().get() == Alliance.Blue) { 
                        
                    double headingX = -pose.getX();
                    double headingY = 5.5531 - pose.getY();
                    speaker = Rotation2d.fromRadians(Math.atan(headingY / headingX));
                } else {

                    double headingX = 16.5418 - pose.getX();
                    double headingY = 5.5531 - pose.getY();
                    speaker = Rotation2d.fromRadians(Math.PI + Math.atan(headingY / headingX));
                }
            }

            this.swerve.driveHeadingLock(translation, speaker);
            return;
        }

        double omega = -this.omega.getAsDouble() * Constants.SwerveConstants.TURN_CONSTANT * Constants.SwerveConstants.MAX_STEER_SPEED;        
        this.swerve.driveAngularVelocity(translation.times(Constants.SwerveConstants.MAX_DRIVE_SPEED), omega);
    }

    @Override
    public boolean isFinished () { return false; }
}
