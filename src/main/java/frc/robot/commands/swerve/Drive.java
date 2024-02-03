package frc.robot.commands.swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Constants;
import swervelib.math.SwerveMath;

public class Drive extends Command {
    
    private final Swerve swerve;
    private final DoubleSupplier vx, vy;
    private final DoubleSupplier omega, headingLock;

    public Drive (Swerve swerve, DoubleSupplier vx, DoubleSupplier vy, DoubleSupplier omega, DoubleSupplier headingLock) {

        this.swerve = swerve;

        this.vx = vx;
        this.vy = vy;

        this.omega = omega;
        this.headingLock = headingLock;

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
        translation = SwerveMath.limitVelocity(
            translation, this.swerve.getFieldVelocity(), this.swerve.getPose(), 
            Constants.RobotConstants.LOOP_TIME, Constants.RobotConstants.ROBOT_MASS, 
            Constants.RobotConstants.WEIGHT_DISTRIBUTION, this.swerve.getConfiguration()
        );

        double headingX = 0.0;
        double headingY = 0.0;
        double headingLock = this.headingLock.getAsDouble();

        if (headingLock != -1.0) {

            if (headingLock >= 45 && headingLock <= 135) { headingX = -1.0; }
            else if (headingLock >= 225 && headingLock <= 315) { headingX = 1.0; }

            if (headingLock >= 315 || headingLock <= 45) { headingY = 1.0; }
            else if (headingLock >= 135 && headingLock <= 225) { headingY = -1.0; }

            this.swerve.driveHeadingLock(translation, new Rotation2d(headingX, headingY));
            return;
        }

        double omega = -this.omega.getAsDouble() * Constants.SwerveConstants.TURN_CONSTANT * Constants.SwerveConstants.MAX_STEER_SPEED;        
        this.swerve.driveAngularVelocity(translation.times(Constants.SwerveConstants.MAX_DRIVE_SPEED), omega);
    }

    @Override
    public boolean isFinished () { return false; }
}
