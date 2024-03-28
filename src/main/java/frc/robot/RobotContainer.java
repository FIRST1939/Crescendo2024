package frc.robot;

import java.io.IOException;
import java.util.function.IntSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.Controller;
import frc.lib.Controller.Actions;
import frc.lib.StateMachine;
import frc.lib.leds.AddressableLEDs;
import frc.lib.leds.patterns.Blue;
import frc.lib.leds.patterns.Green;
import frc.lib.leds.patterns.Yellow;
import frc.robot.commands.IdleMode;
import frc.robot.commands.arm.LockArm;
import frc.robot.commands.arm.PivotArm;
import frc.robot.commands.auto.AutoEjectNote;
import frc.robot.commands.auto.AutoScoreNote;
import frc.robot.commands.elevator.LockElevator;
import frc.robot.commands.elevator.RaiseElevator;
import frc.robot.commands.indexer.DropNote;
import frc.robot.commands.indexer.FeedNote;
import frc.robot.commands.indexer.HoldAmpNote;
import frc.robot.commands.indexer.HoldSpeakerNote;
import frc.robot.commands.indexer.IdleIndexer;
import frc.robot.commands.indexer.IndexAmpNote;
import frc.robot.commands.indexer.IndexSpeakerNote;
import frc.robot.commands.indexer.ReverseNote;
import frc.robot.commands.intake.IdleIntake;
import frc.robot.commands.intake.IntakeNote;
import frc.robot.commands.intake.OutakeNote;
import frc.robot.commands.shooter.IdleShooter;
import frc.robot.commands.shooter.ShootNote;
import frc.robot.commands.swerve.Drive;
import frc.robot.commands.swerve.TrackAprilTags;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Swerve.Target;
import frc.robot.util.Alerts;
import frc.robot.util.Constants;
import frc.robot.util.Constants.IdleBehavior;
import frc.robot.util.Sensors;

public class RobotContainer {

    private Swerve swerve;
    private Limelight limelight;
    
    private Intake intake;
    private Elevator elevator;
    private Indexer indexer;
    private Arm arm;
    private Shooter shooter;
    private AddressableLEDs leds;

    private StateMachine intakeStateMachine;
    private StateMachine elevatorStateMachine;
    private StateMachine indexerStateMachine;
    private StateMachine armStateMachine;
    private StateMachine shooterStateMachine;

    private Controller driverOne;
    private Controller driverTwo;

    private SendableChooser<Command> autonomousChooser;

    public RobotContainer () {

        try { this.swerve = new Swerve(); }
        catch (IOException ioException) {}
        this.limelight = new Limelight();

        this.intake = new Intake();
        this.elevator = new Elevator();
        this.indexer = new Indexer();
        this.arm = new Arm();
        this.shooter = new Shooter();
        this.leds = new AddressableLEDs(Constants.RobotConstants.LEDS, Constants.RobotConstants.LED_COUNT);

        this.intakeStateMachine = new StateMachine(Alerts.intakeStateMachine, this.intake);
        this.elevatorStateMachine = new StateMachine(Alerts.elevatorStateMachine, this.elevator);
        this.indexerStateMachine = new StateMachine(Alerts.indexerStateMachine, this.indexer);
        this.armStateMachine = new StateMachine(Alerts.armStateMachine, this.arm);
        this.shooterStateMachine = new StateMachine(Alerts.shooterStateMachine, this.shooter);

        this.driverOne = new Controller(0);
        this.driverTwo = new Controller(1);
        
        this.configureCommands();
        this.initializePathPlanner();
    }

    private void configureCommands () {

        IntSupplier allianceOriented = () -> {

            if (!DriverStation.getAlliance().isPresent()) { return -1; }
            return DriverStation.getAlliance().get() == Alliance.Red ? 1 : -1;
        };

        this.swerve.setDefaultCommand(new Drive(
            this.swerve, 
            () -> MathUtil.applyDeadband(this.driverOne.getHID().getLeftY() * allianceOriented.getAsInt(), Constants.SwerveConstants.TRANSLATION_DEADBAND),
            () -> MathUtil.applyDeadband(this.driverOne.getHID().getLeftX() * allianceOriented.getAsInt(), Constants.SwerveConstants.TRANSLATION_DEADBAND),
            () -> MathUtil.applyDeadband(this.driverOne.getHID().getRightX(), Constants.SwerveConstants.OMEGA_DEADBAND), 
            () -> this.driverOne.getHID().getLeftBumperPressed(),
            () -> this.driverOne.getHID().getRightBumperPressed()
        ));

        this.limelight.setDefaultCommand(new TrackAprilTags(this.swerve, this.limelight));

        this.driverOne.x().onTrue(new InstantCommand(this.swerve::zeroGyro, this.swerve));
        this.driverOne.leftBumper().whileTrue(new RepeatCommand(new InstantCommand(this.swerve::lock, this.swerve)));

        this.driverTwo.povLeft().onTrue(new InstantCommand(() -> {

            Constants.ArmConstants.PIVOT_POSITION = 59.0;
            Constants.ShooterConstants.TOP_SHOOT_SPEED = 800.0;
            Constants.ShooterConstants.BOTTOM_SHOOT_SPEED = 800.0;
        }));

        this.driverTwo.povDown().onTrue(new InstantCommand(() -> {

            Constants.ArmConstants.PIVOT_POSITION = 34.0;
            Constants.ShooterConstants.TOP_SHOOT_SPEED = 800.0;
            Constants.ShooterConstants.BOTTOM_SHOOT_SPEED = 800.0;
        }));

        this.driverTwo.povUp().onTrue(new InstantCommand(() -> {

            Constants.ArmConstants.PIVOT_POSITION = 28.0;
            Constants.ShooterConstants.TOP_SHOOT_SPEED = 1000.0;
            Constants.ShooterConstants.BOTTOM_SHOOT_SPEED = 1000.0;
        }));


        this.driverTwo.povRight().onTrue(new InstantCommand(() -> {

            Constants.ArmConstants.PIVOT_POSITION = 30.0;
            Constants.ShooterConstants.TOP_SHOOT_SPEED = 900.0;
            Constants.ShooterConstants.BOTTOM_SHOOT_SPEED = 900.0;
        }));

        this.driverTwo.x().onTrue(new InstantCommand(() -> this.transferObjective(Target.SPEAKER)));
        this.driverTwo.a().onTrue(new InstantCommand(() -> this.transferObjective(Target.AMP)));
        this.driverTwo.y().onTrue(new InstantCommand(() -> this.transferObjective(Target.STAGE)));
    }

    public void initializeStateMachines (Class<? extends Command> intakeState, Class<? extends Command> elevatorState, Class<? extends Command> indexerState, Class<? extends Command> armState, Class<? extends Command> shooterState) {

        this.intakeStateMachine.activateState(intakeState);
        this.elevatorStateMachine.activateState(elevatorState);
        this.indexerStateMachine.activateState(indexerState);
        this.armStateMachine.activateState(armState);
        this.shooterStateMachine.activateState(shooterState);
    }
    
    public void runStateMachines () {

        Class<? extends Command> intakeState = this.intakeStateMachine.getCurrentState();
        Class<? extends Command> elevatorState = this.elevatorStateMachine.getCurrentState();
        Class<? extends Command> indexerState = this.indexerStateMachine.getCurrentState();
        Class<? extends Command> armState = this.armStateMachine.getCurrentState();
        Class<? extends Command> shooterState = this.shooterStateMachine.getCurrentState();
        
        boolean intakeFinished = this.intakeStateMachine.currentCommandFinished();
        boolean indexerFinished = this.indexerStateMachine.currentCommandFinished();
        boolean armFinished = this.armStateMachine.currentCommandFinished();
        boolean shooterFinished = this.shooterStateMachine.currentCommandFinished();

        if (intakeState != IdleIndexer.class && (!Sensors.getIndexerStartBeam() || !Sensors.getIndexerEndBeam())) {

            this.intakeStateMachine.activateState(IdleIntake.class);
        }

        if (indexerState == IndexSpeakerNote.class && indexerFinished) {

            this.intakeStateMachine.activateState(IdleIntake.class);
            this.indexerStateMachine.activateState(HoldSpeakerNote.class);
        }

        if (indexerState == IndexAmpNote.class && indexerFinished) {

            this.intakeStateMachine.activateState(IdleIntake.class);
            this.indexerStateMachine.activateState(HoldAmpNote.class);
        }

        if (indexerState == FeedNote.class && indexerFinished) {

            this.indexerStateMachine.activateState(IdleIndexer.class);
        }

        if (indexerState == DropNote.class && indexerFinished) {

            this.indexerStateMachine.activateState(IdleIndexer.class);
        }

        boolean leftBumper = this.driverTwo.getHID().getLeftBumperPressed();
        boolean rightBumper = this.driverTwo.getHID().getRightBumperPressed();
        Actions leftTrigger = this.driverTwo.getLeftTrigger();
        Actions rightTrigger = this.driverTwo.getRightTrigger();

        if (leftBumper) {

            if (intakeState != OutakeNote.class) { this.intakeStateMachine.activateState(OutakeNote.class); }
            else { this.intakeStateMachine.activateState(IdleIntake.class); }

            if (indexerState != ReverseNote.class) { this.indexerStateMachine.activateState(ReverseNote.class); }
            else { this.indexerStateMachine.activateState(IdleIndexer.class); }
        }

        if (rightBumper) {

            if (intakeState != IntakeNote.class) { this.intakeStateMachine.activateState(IntakeNote.class); }
            else { this.intakeStateMachine.activateState(IdleIntake.class); }

            if (Swerve.target == Target.SPEAKER) {

                if (indexerState != IndexSpeakerNote.class) { this.indexerStateMachine.activateState(IndexSpeakerNote.class); }
                else { this.indexerStateMachine.activateState(IdleIndexer.class); }
            }

            if (Swerve.target == Target.AMP) {

                if (indexerState != IndexAmpNote.class) { this.indexerStateMachine.activateState(IndexAmpNote.class); }
                else { this.indexerStateMachine.activateState(IdleIndexer.class); }
            }
        }

        if (leftTrigger == Actions.PRESS) {

            this.elevatorStateMachine.activateState(LockElevator.class);
            this.armStateMachine.activateState(LockArm.class);
            this.shooterStateMachine.activateState(IdleShooter.class);
        }

        if (rightTrigger == Actions.PRESS) {

            if (Swerve.target == Target.SPEAKER) {

                this.armStateMachine.activateState(PivotArm.class);
                this.shooterStateMachine.activateState(ShootNote.class);
            }

            if (Swerve.target == Target.AMP) {

                this.elevatorStateMachine.activateState(RaiseElevator.class);
            }
        }

        if (rightTrigger == Actions.RELEASE) {

            if (Swerve.target == Target.SPEAKER) {

                if (this.arm.atPosition() && this.shooter.atSpeed()) {

                    this.indexerStateMachine.activateState(FeedNote.class);
                }
            }

            if (Swerve.target == Target.AMP) {

                if (this.elevator.atHeight()) {

                    this.indexerStateMachine.activateState(DropNote.class);
                }
            }
        }
    }

    public void transferObjective (Target target) {

        Class<? extends Command> elevatorState = this.elevatorStateMachine.getCurrentState();
        Class<? extends Command> indexerState = this.indexerStateMachine.getCurrentState();

        if (Swerve.target == Target.AMP && target == Target.SPEAKER) {

            if (indexerState == IndexAmpNote.class) { this.indexerStateMachine.activateState(IndexSpeakerNote.class); }
            if (indexerState == HoldAmpNote.class && elevatorState == LockElevator.class) { this.indexerStateMachine.activateState(IndexSpeakerNote.class); }
        } else if (Swerve.target != Target.STAGE && target == Target.STAGE) {

            this.elevatorStateMachine.activateState(RaiseElevator.class);
            this.armStateMachine.activateState(LockArm.class);
        } else if (Swerve.target == Target.STAGE && target != Target.STAGE) {

            this.elevatorStateMachine.activateState(LockElevator.class);
        }

        Swerve.target = target;
    }

    public void runLEDs () {

        if (Swerve.target == Target.SPEAKER) { this.leds.setPattern(Blue.class); }
        else if (Swerve.target == Target.AMP) { this.leds.setPattern(Green.class); }
        else if (Swerve.target == Target.STAGE) { this.leds.setPattern(Yellow.class); }
    }

    public void initializePathPlanner () {

        NamedCommands.registerCommand("SubShoot", new SequentialCommandGroup(
            new InstantCommand(() -> {

                Constants.ArmConstants.PIVOT_POSITION = 59.0;
                Constants.ShooterConstants.TOP_SHOOT_SPEED = 800.0;
                Constants.ShooterConstants.BOTTOM_SHOOT_SPEED = 800.0;
            }),
            new AutoScoreNote(this.armStateMachine, this.shooterStateMachine)
        ));

        NamedCommands.registerCommand("FLShoot", new SequentialCommandGroup(
            new InstantCommand(() -> {

                Constants.ArmConstants.PIVOT_POSITION = 34.0;
                Constants.ShooterConstants.TOP_SHOOT_SPEED = 800.0;
                Constants.ShooterConstants.BOTTOM_SHOOT_SPEED = 800.0;
            }),
            new AutoScoreNote(this.armStateMachine, this.shooterStateMachine)
        ));

        NamedCommands.registerCommand("StageShoot", new SequentialCommandGroup(
            new InstantCommand(() -> {

                Constants.ArmConstants.PIVOT_POSITION = 28.0;
                Constants.ShooterConstants.TOP_SHOOT_SPEED = 1000.0;
                Constants.ShooterConstants.BOTTOM_SHOOT_SPEED = 1000.0;
            }),
            new AutoScoreNote(this.armStateMachine, this.shooterStateMachine)
        ));

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

        boolean intakeFinished = this.intakeStateMachine.currentCommandFinished();
        boolean indexerFinished = this.indexerStateMachine.currentCommandFinished();
        boolean armFinished = this.armStateMachine.currentCommandFinished();
        boolean shooterFinished = this.shooterStateMachine.currentCommandFinished();

        if (intakeState == IntakeNote.class && intakeFinished) { this.intakeStateMachine.activateState(IdleIntake.class); }
        if (indexerState == IndexSpeakerNote.class && indexerFinished) { this.indexerStateMachine.activateState(HoldSpeakerNote.class); }

        if (indexerState == FeedNote.class && indexerFinished) { 
            
            this.intakeStateMachine.activateState(IntakeNote.class);
            this.indexerStateMachine.activateState(IndexSpeakerNote.class);
        }

        if (armState == PivotArm.class && armFinished) { this.armStateMachine.activateState(LockArm.class); }
        if (shooterState == ShootNote.class && shooterFinished) { this.shooterStateMachine.activateState(IdleShooter.class); }

        if (armState == PivotArm.class && shooterState == ShootNote.class) {

            if (this.arm.atPosition() && this.shooter.atSpeed()) { 
                
                this.indexerStateMachine.activateState(FeedNote.class); 
            }
        }
    }

    public Command getAutonomousCommand () { return this.autonomousChooser.getSelected(); }

    public void setIdleModes (IdleBehavior swerveIdle, IdleBehavior intakeIdle, IdleBehavior elevatorIdle, IdleBehavior indexerIdle, IdleBehavior armIdle, IdleBehavior shooterIdle) {

        new IdleMode(
            this.swerve, swerveIdle, 
            this.intake, intakeIdle, 
            this.elevator, elevatorIdle,
            this.indexer, indexerIdle, 
            this.arm, armIdle, 
            this.shooter, shooterIdle
        ).schedule();
    }
}
