package frc.robot;

import java.io.IOException;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.lib.Controller;
import frc.lib.StateMachine;
import frc.robot.commands.IdleMode;
import frc.robot.commands.arm.LockArm;
import frc.robot.commands.arm.PivotArm;
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

    private StateMachine intakeStateMachine;
    private StateMachine indexerStateMachine;
    private StateMachine armStateMachine;
    private StateMachine shooterStateMachine;

    private Controller driverOne;
    private Controller driverTwo;

    public RobotContainer () {

        try { this.swerve = new Swerve(); }
        catch (IOException ioException) {}
        //this.limelight = new Limelight();

        this.intake = new Intake();
        this.indexer = new Indexer();
        this.arm = new Arm();
        this.shooter = new Shooter();

        this.intakeStateMachine = new StateMachine(Alerts.intakeStateMachine, this.intake);
        this.indexerStateMachine = new StateMachine(Alerts.indexerStateMachine, this.indexer);
        this.armStateMachine = new StateMachine(Alerts.armStateMachine, this.arm);
        this.shooterStateMachine = new StateMachine(Alerts.shooterStateMachine, this.shooter);

        this.driverOne = new Controller(0);
        this.driverTwo = new Controller(1);
        this.configureCommands();
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
        }

        if (this.driverTwo.getHID().getLeftBumperPressed()) {

            if (indexerState == IndexNote.class || indexerState == HoldNote.class) {

                this.intakeStateMachine.activateState(OutakeNote.class);
                this.indexerStateMachine.activateState(ReverseNote.class);
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
                }
            }
        }
    }

    public Command getAutonomousCommand () { return this.swerve.getAutonomousCommand(); }

    public void setIdleModes (IdleBehavior swerveIdle, IdleBehavior intakeIdle, IdleBehavior indexerIdle, IdleBehavior armIdle, IdleBehavior shooterIdle) {

        new IdleMode(
            this.swerve, swerveIdle, 
            this.intake, intakeIdle, 
            this.indexer, indexerIdle, 
            this.arm, armIdle, 
            this.shooter, shooterIdle
        ).schedule();
    }
}
