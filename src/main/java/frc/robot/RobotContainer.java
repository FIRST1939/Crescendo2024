package frc.robot;

import java.io.IOException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.lib.Controller;
import frc.lib.StateMachine;
import frc.robot.commands.IdleMode;
import frc.robot.commands.ShotFeedback;
import frc.robot.commands.arm.LockArm;
import frc.robot.commands.arm.PivotArm;
import frc.robot.commands.auto.AutoEjectNote;
import frc.robot.commands.auto.AutoScoreNote;
import frc.robot.commands.indexer.EjectNote;
import frc.robot.commands.indexer.FeedNote;
import frc.robot.commands.indexer.HoldNote;
import frc.robot.commands.indexer.IdleIndexer;
import frc.robot.commands.indexer.IndexNote;
import frc.robot.commands.indexer.ReverseNote;
import frc.robot.commands.intake.IdleIntake;
import frc.robot.commands.intake.IntakeNote;
import frc.robot.commands.intake.OutakeNote;
import frc.robot.commands.shooter.IdleShooter;
import frc.robot.commands.shooter.ShootNote;
import frc.robot.commands.swerve.Drive;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Logging;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Alerts;
import frc.robot.util.Constants;
import frc.robot.util.Constants.IdleBehavior;

public class RobotContainer {

    private Swerve swerve;
    private Limelight limelight;
    
    private Intake intake;
    private Indexer indexer;
    private Arm arm;
    private Shooter shooter;
    private Logging logging;

    private StateMachine intakeStateMachine;
    private StateMachine indexerStateMachine;
    private StateMachine armStateMachine;
    private StateMachine shooterStateMachine;

    private Controller driverOne;
    private Controller driverTwo;

    private SendableChooser<Command> autonomousChooser;

    public RobotContainer () {

        try { this.swerve = new Swerve(); }
        catch (IOException ioException) {}
        //this.limelight = new Limelight();

        this.intake = new Intake();
        this.indexer = new Indexer();
        this.arm = new Arm();
        this.shooter = new Shooter();

        this.logging = new Logging(
            () -> this.arm.getPosition(), 
            () -> this.shooter.getTopVelocity(), 
            () -> this.shooter.getBottomVelocity(),
            () -> this.indexer.getBeamBreak()
        );

        this.intakeStateMachine = new StateMachine(Alerts.intakeStateMachine, this.intake);
        this.indexerStateMachine = new StateMachine(Alerts.indexerStateMachine, this.indexer);
        this.armStateMachine = new StateMachine(Alerts.armStateMachine, this.arm);
        this.shooterStateMachine = new StateMachine(Alerts.shooterStateMachine, this.shooter);

        this.driverOne = new Controller(0);
        this.driverTwo = new Controller(1);
        
        this.configureCommands();
        this.initializePathPlanner();
    }

    private void configureCommands () {

        this.swerve.setDefaultCommand(new Drive(
            this.swerve, 
            () -> MathUtil.applyDeadband(-this.driverOne.getHID().getLeftY(), Constants.SwerveConstants.TRANSLATION_DEADBAND),
            () -> MathUtil.applyDeadband(-this.driverOne.getHID().getLeftX(), Constants.SwerveConstants.TRANSLATION_DEADBAND),
            () -> MathUtil.applyDeadband(this.driverOne.getHID().getRightX(), Constants.SwerveConstants.OMEGA_DEADBAND), 
            () -> this.driverOne.getHID().getPOV()
        ));

        //this.limelight.setDefaultCommand(new TrackAprilTags(this.swerve, this.limelight));

        this.driverOne.x().onTrue(new InstantCommand(this.swerve::zeroGyro, this.swerve));
        this.driverOne.leftBumper().whileTrue(new RepeatCommand(new InstantCommand(this.swerve::lock, this.swerve)));

        this.driverTwo.povUp().onTrue(new InstantCommand(() -> this.arm.manualPivotAdjustment += 0.5));
        this.driverTwo.povDown().onTrue(new InstantCommand(() -> this.arm.manualPivotAdjustment -= 0.5));

        this.driverTwo.x().onTrue(new InstantCommand(() -> SmartDashboard.putString("Target", "Speaker")));
        this.driverTwo.a().onTrue(new InstantCommand(() -> SmartDashboard.putString("Target", "Amp")));
        this.driverTwo.y().onTrue(new InstantCommand(() -> SmartDashboard.putString("Target", "Note")));
        this.driverTwo.b().onTrue(new InstantCommand(() -> SmartDashboard.putString("Target", "Defense")));

        /*
        this.driverTwo.leftBumper().whileTrue(this.intake.getTopQuasistaticRoutine(Direction.kReverse));
        this.driverTwo.rightBumper().whileTrue(this.intake.getTopQuasistaticRoutine(Direction.kForward));

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
        this.driverTwo.leftBumper().whileTrue(this.indexer.getFrontQuasistaticRoutine(Direction.kReverse));
        this.driverTwo.rightBumper().whileTrue(this.indexer.getFrontQuasistaticRoutine(Direction.kForward));
        
        this.driverTwo.leftTrigger().whileTrue(this.indexer.getFrontDynamicRoutine(Direction.kReverse));
        this.driverTwo.rightTrigger().whileTrue(this.indexer.getFrontDynamicRoutine(Direction.kForward));
        */

        /*
        this.driverTwo.leftBumper().whileTrue(this.indexer.getBackQuasistaticRoutine(Direction.kReverse));
        this.driverTwo.rightBumper().whileTrue(this.indexer.getBackQuasistaticRoutine(Direction.kForward));
        
        this.driverTwo.leftTrigger().whileTrue(this.indexer.getBackDynamicRoutine(Direction.kReverse));
        this.driverTwo.rightTrigger().whileTrue(this.indexer.getBackDynamicRoutine(Direction.kForward));
        */

        /*
        this.driverTwo.leftBumper().whileTrue(this.arm.getQuasistaticRoutine(Direction.kReverse));
        this.driverTwo.rightBumper().whileTrue(this.arm.getQuasistaticRoutine(Direction.kForward));
        
        this.driverTwo.leftTrigger().whileTrue(this.arm.getDynamicRoutine(Direction.kReverse));
        this.driverTwo.rightTrigger().whileTrue(this.arm.getDynamicRoutine(Direction.kForward));
        */

        /*
        this.driverTwo.leftBumper().whileTrue(this.shooter.getQuasistaticRoutine(Direction.kReverse));
        this.driverTwo.rightBumper().whileTrue(this.shooter.getQuasistaticRoutine(Direction.kForward));

        this.driverTwo.leftTrigger().whileTrue(this.shooter.getDynamicRoutine(Direction.kReverse));
        this.driverTwo.rightTrigger().whileTrue(this.shooter.getDynamicRoutine(Direction.kForward));
        */
    }

    public void initializeStateMachines (Class<? extends Command> intakeState, Class<? extends Command> indexerState, Class<? extends Command> armState, Class<? extends Command> shooterState) {

        this.intakeStateMachine.activateState(intakeState);
        this.indexerStateMachine.activateState(indexerState);
        this.armStateMachine.activateState(armState);
        this.shooterStateMachine.activateState(shooterState);
    }
    
    public void runStateMachines () {

        Class<? extends Command> intakeState = this.intakeStateMachine.getCurrentState();
        Class<? extends Command> indexerState = this.indexerStateMachine.getCurrentState();
        Class<? extends Command> armState = this.armStateMachine.getCurrentState();
        Class<? extends Command> shooterState = this.shooterStateMachine.getCurrentState();

        if (intakeState == IntakeNote.class && this.indexer.noteContained()) { this.intakeStateMachine.activateState(IdleIntake.class); }
        if (indexerState == IndexNote.class && this.indexer.noteIndexed()) { this.indexerStateMachine.activateState(HoldNote.class); }
        if (indexerState == FeedNote.class && this.indexer.noteFed()) { 
            
            this.indexerStateMachine.activateState(IdleIndexer.class);
            this.armStateMachine.activateState(LockArm.class);
            this.shooterStateMachine.activateState(IdleShooter.class);

            this.logging.setLogging(false);
            new ShotFeedback(this.logging).schedule();
        }

        if (this.driverTwo.getHID().getLeftBumperPressed()) {

            if (indexerState == IndexNote.class || indexerState == HoldNote.class) {

                this.intakeStateMachine.activateState(OutakeNote.class);
                this.indexerStateMachine.activateState(ReverseNote.class);
            } else if (intakeState == OutakeNote.class || indexerState == ReverseNote.class) {

                this.intakeStateMachine.activateState(IdleIntake.class);
                this.indexerStateMachine.activateState(IdleIndexer.class);
            }
        }

        if (this.driverTwo.getHID().getRightBumperPressed()) {

            if (indexerState == IdleIndexer.class || indexerState == IdleIndexer.class) {

                this.intakeStateMachine.activateState(IntakeNote.class);
                this.indexerStateMachine.activateState(IndexNote.class);
            } else if (indexerState == HoldNote.class) {

                if (armState == LockArm.class || shooterState == IdleShooter.class) { 
                    
                    this.armStateMachine.activateState(PivotArm.class);
                    this.shooterStateMachine.activateState(ShootNote.class); 
                } else if (this.arm.atPosition() && this.shooter.atSpeed()) { 
                    
                    this.indexerStateMachine.activateState(FeedNote.class); 
                    this.logging.setLogging(true);
                }
            } else if (intakeState == IntakeNote.class || indexerState == IndexNote.class) {

                this.intakeStateMachine.activateState(IdleIntake.class);
                this.indexerStateMachine.activateState(IdleIndexer.class);
            }
        }

        if (this.driverTwo.getRightTriggerAxis() > 0.5) {

            if (!(intakeState == EjectNote.class || indexerState == EjectNote.class || shooterState == EjectNote.class)) {

                this.intakeStateMachine.activateState(frc.robot.commands.intake.EjectNote.class);
                this.indexerStateMachine.activateState(frc.robot.commands.indexer.EjectNote.class);
                this.shooterStateMachine.activateState(frc.robot.commands.shooter.EjectNote.class);
            }
        } else {

            if (intakeState == frc.robot.commands.intake.EjectNote.class &&
                indexerState == frc.robot.commands.indexer.EjectNote.class &&
                shooterState == frc.robot.commands.shooter.EjectNote.class) {

                this.intakeStateMachine.activateState(IdleIntake.class);
                this.indexerStateMachine.activateState(IdleIndexer.class);
                this.shooterStateMachine.activateState(IdleShooter.class);
            }
        }
    }

    public void initializePathPlanner () {

        NamedCommands.registerCommand("Score", new AutoScoreNote(this.armStateMachine, this.shooterStateMachine));
        NamedCommands.registerCommand("Eject", new AutoEjectNote(this.intakeStateMachine, this.indexerStateMachine, this.shooterStateMachine));

        AutoBuilder.configureHolonomic(
            this.swerve::getPose, this.swerve::resetOdometry,
            this.swerve::getRobotVelocity, this.swerve::setChassisSpeeds,
            new HolonomicPathFollowerConfig(
                new PIDConstants(5.0, 0.0, 0.0),
                new PIDConstants(
                    this.swerve.getHeadingPIDFConfig().p,
                    this.swerve.getHeadingPIDFConfig().i,
                    this.swerve.getHeadingPIDFConfig().d
                ),
                Constants.SwerveConstants.MAX_DRIVE_SPEED,
                this.swerve.getConfiguration().getDriveBaseRadiusMeters(),
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
            this.swerve
        );

        this.autonomousChooser = AutoBuilder.buildAutoChooser();
        Shuffleboard.getTab("Autonomous").add("Autonomous Chooser", this.autonomousChooser);
    }

    public void runAutoStateMachines () {

        Class<? extends Command> intakeState = this.intakeStateMachine.getCurrentState();
        Class<? extends Command> indexerState = this.indexerStateMachine.getCurrentState();
        Class<? extends Command> armState = this.armStateMachine.getCurrentState();
        Class<? extends Command> shooterState = this.shooterStateMachine.getCurrentState();

        if (intakeState == IntakeNote.class && this.indexer.noteContained()) { this.intakeStateMachine.activateState(IdleIntake.class); }
        if (indexerState == IndexNote.class && this.indexer.noteIndexed()) { this.indexerStateMachine.activateState(HoldNote.class); }
        if (indexerState == FeedNote.class && this.indexer.noteFed()) { 
            
            this.intakeStateMachine.activateState(IntakeNote.class);
            this.indexerStateMachine.activateState(IndexNote.class);
            this.armStateMachine.activateState(LockArm.class);
            this.shooterStateMachine.activateState(IdleShooter.class);
        }

        if (armState == PivotArm.class && shooterState == ShootNote.class) {

            if (this.arm.atPosition() && this.shooter.atSpeed()) { 
                
                this.indexerStateMachine.activateState(FeedNote.class); 
            }
        }
    }

    public Command getAutonomousCommand () { return this.autonomousChooser.getSelected(); }

    public void setIdleModes (IdleBehavior swerveIdle, IdleBehavior intakeIdle, IdleBehavior indexerIdle, IdleBehavior armIdle, IdleBehavior shooterIdle) {

        new IdleMode(
            this.swerve, swerveIdle, 
            this.intake, intakeIdle, 
            this.indexer, indexerIdle, 
            this.arm, armIdle, 
            this.shooter, shooterIdle
        ).schedule();
    }

    public void saveLog () { this.logging.saveLog(); }
}
