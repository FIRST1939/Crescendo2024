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
import frc.robot.subsystems.Swerve.Target;
import frc.robot.util.Constants;

public class Drive extends Command {
    
    private final Swerve swerve;
    private final DoubleSupplier vx, vy;
    private final DoubleSupplier omega;

    private boolean sourceHeadingLock = false;
    private boolean objectiveHeadingLock = false;
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

        if (Swerve.target == Target.SPEAKER && Constants.SwerveConstants.REGRESSION) {

            double distance = this.swerve.getSpeakerDistance();
            double angle = 99.166 * Math.pow(Math.E, -0.688959 * distance) + 21.4836;

            Constants.ArmConstants.PIVOT_POSITION = angle;
            Constants.ShooterConstants.TOP_SHOOT_SPEED = 1140.0;
            Constants.ShooterConstants.BOTTOM_SHOOT_SPEED = 1140.0;
        }

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

        if (this.leftBumper.getAsBoolean()) { this.sourceHeadingLock = true; }
        if (this.rightBumper.getAsBoolean()) { this.objectiveHeadingLock = true; }

        if (this.omega.getAsDouble() != 0.0) {

            this.sourceHeadingLock = false;
            this.objectiveHeadingLock = false;
        }

        if (this.sourceHeadingLock) {

            Rotation2d source = new Rotation2d();

            if (DriverStation.getAlliance().get() == Alliance.Blue) { 
                        
                source = new Rotation2d(-60.0); 
            } else {

                source = new Rotation2d(240.0);
            }

            this.swerve.driveHeadingLock(translation, source);
            return;
        }

        if (this.objectiveHeadingLock) {

            Rotation2d objective = new Rotation2d();

            if (Swerve.target == Target.SPEAKER) {

                Pose2d pose = this.swerve.getPose();
                
                if (DriverStation.getAlliance().isPresent()) {

                    if (DriverStation.getAlliance().get() == Alliance.Blue) { 
                        
                        double headingX = -pose.getX();
                        double headingY = 5.5531 - pose.getY();
                        objective = Rotation2d.fromRadians(Math.atan(headingY / headingX));
                    } else {

                        double headingX = 16.5418 - pose.getX();
                        double headingY = 5.5531 - pose.getY();
                        objective = Rotation2d.fromRadians(Math.PI + Math.atan(headingY / headingX));
                    }
                }
            } else if (Swerve.target == Target.AMP) { 
                
                objective = new Rotation2d(90.0); 
            }

            this.swerve.driveHeadingLock(translation, objective);
            return;
        }

        double omega = -this.omega.getAsDouble() * Constants.SwerveConstants.TURN_CONSTANT * Constants.SwerveConstants.MAX_STEER_SPEED;        
        this.swerve.driveAngularVelocity(translation.times(Constants.SwerveConstants.MAX_DRIVE_SPEED), omega);
    }

    @Override
    public boolean isFinished () { return false; }
}
