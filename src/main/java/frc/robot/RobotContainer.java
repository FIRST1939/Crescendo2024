package frc.robot;

import java.io.IOException;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.Controller;
import frc.robot.commands.swerve.BrakeMode;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class RobotContainer {

    private Swerve swerve;
    private Limelight limelight;
    
    private Intake intake;
    private Indexer indexer;
    private Arm arm;
    private Shooter shooter;

    private Controller driverOne;
    private Controller driverTwo;

    public RobotContainer () {

        try { this.swerve = new Swerve(); }
        catch (IOException ioException) {}
        this.limelight = new Limelight();

        this.intake = new Intake();
        this.indexer = new Indexer();
        this.arm = new Arm();
        this.shooter = new Shooter();
        
        this.driverOne = new Controller(0);
        this.driverTwo = new Controller(1);

        this.configureCommands();
    }

    private void configureCommands () {

        /*
        this.swerve.setDefaultCommand(new Drive(
            this.swerve, 
            () -> MathUtil.applyDeadband(-this.driverOne.getHID().getLeftY(), Constants.SwerveConstants.TRANSLATION_DEADBAND),
            () -> MathUtil.applyDeadband(-this.driverOne.getHID().getLeftX(), Constants.SwerveConstants.TRANSLATION_DEADBAND), 
            () -> MathUtil.applyDeadband(this.driverOne.getHID().getRightX(), Constants.SwerveConstants.OMEGA_DEADBAND), 
            () -> this.driverOne.getHID().getPOV()
        ));

        this.limelight.setDefaultCommand(new TrackAprilTags(this.swerve, this.limelight));

        this.driverOne.x().onTrue(new InstantCommand(this.swerve::zeroGyro, this.swerve));
        this.driverOne.leftBumper().whileTrue(new RepeatCommand(new InstantCommand(this.swerve::lock, this.swerve)));
        */

        /*
        this.driverTwo.leftBumper().whileTrue(this.intake.getTopQuasistaticRoutine(Direction.kReverse));
        this.driverTwo.rightBumper().whileTrue(this.intake.getBottomQuasistaticRoutine(Direction.kForward));

        this.driverTwo.leftTrigger().whileTrue(this.intake.getTopDynamicRoutine(Direction.kReverse));
        this.driverTwo.rightTrigger().whileTrue(this.intake.getTopDynamicRoutine(Direction.kForward));
        */

        /*
        this.driverTwo.leftBumper().whileTrue(this.intake.getBottomQuasistaticRoutine(Direction.kReverse));
        this.driverTwo.rightBumper().whileTrue(this.intake.getBottomQuasistaticRoutine(Direction.kForward));

        this.driverTwo.leftTrigger().whileTrue(this.intake.getBottomDynamicRoutine(Direction.kReverse));
        this.driverTwo.rightTrigger().whileTrue(this.intake.getBottomDynamicRoutine(Direction.kForward));
        */

        /*
        this.driverTwo.leftBumper().whileTrue(this.indexer.getQuasistaticRoutine(Direction.kReverse));
        this.driverTwo.rightBumper().whileTrue(this.indexer.getQuasistaticRoutine(Direction.kForward));
        
        this.driverTwo.leftTrigger().whileTrue(this.indexer.getDynamicRoutine(Direction.kReverse));
        this.driverTwo.rightTrigger().whileTrue(this.indexer.getDynamicRoutine(Direction.kForward));
        */

        /*
        this.driverTwo.leftBumper().whileTrue(this.arm.getQuasistaticRoutine(Direction.kReverse));
        this.driverTwo.rightBumper().whileTrue(this.arm.getQuasistaticRoutine(Direction.kForward));
        
        this.driverTwo.leftTrigger().whileTrue(this.arm.getDynamicRoutine(Direction.kReverse));
        this.driverTwo.rightTrigger().whileTrue(this.indexer.getDynamicRoutine(Direction.kForward));
        */

        /*
        this.driverTwo.leftBumper().whileTrue(this.shooter.getQuasistaticRoutine(Direction.kReverse));
        this.driverTwo.leftBumper().whileTrue(this.shooter.getQuasistaticRoutine(Direction.kForward));

        this.driverTwo.leftTrigger().whileTrue(this.shooter.getDynamicRoutine(Direction.kReverse));
        this.driverTwo.rightTrigger().whileTrue(this.shooter.getDynamicRoutine(Direction.kForward));
        */
    }

    public Command getAutonomousCommand () { return this.swerve.getAutonomousCommand(); }
    public void setBrakeMode (boolean brake) { new BrakeMode(this.swerve, brake).schedule(); }
}
