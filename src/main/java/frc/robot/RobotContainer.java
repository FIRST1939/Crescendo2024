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
import frc.lib.Controller;
import frc.lib.StateMachine;
import frc.lib.leds.AddressableLEDs;
import frc.lib.leds.patterns.Blue;
import frc.lib.leds.patterns.BlueBlink;
import frc.lib.leds.patterns.Green;
import frc.lib.leds.patterns.GreenBlink;
import frc.lib.leds.patterns.Note;
import frc.lib.leds.patterns.NoteBlink;
import frc.lib.leds.patterns.Rainbow;
import frc.lib.leds.patterns.Yellow;
import frc.lib.leds.patterns.YellowBlink;
import frc.robot.commands.IdleMode;
import frc.robot.commands.arm.LockArm;
import frc.robot.commands.arm.PivotArm;
import frc.robot.commands.auto.AutoEjectNote;
import frc.robot.commands.auto.AutoScoreNote;
import frc.robot.commands.elevator.LockElevator;
import frc.robot.commands.elevator.ManualElevator;
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

    private boolean regression = true;
    private Objective objective = Objective.SPEAKER;
    
    enum Objective {
        SPEAKER,
        AMP,
        TRAP,
        STAGE
    }

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

        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {

            this.driverOne.a().whileTrue(this.swerve.pathfind(Constants.SwerveConstants.BLUE_LEFT_TRAP));
            this.driverOne.b().whileTrue(this.swerve.pathfind(Constants.SwerveConstants.BLUE_CENTER_TRAP));
            this.driverOne.y().whileTrue(this.swerve.pathfind(Constants.SwerveConstants.BLUE_RIGHT_TRAP));
        } else if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {

            this.driverOne.a().whileTrue(this.swerve.pathfind(Constants.SwerveConstants.RED_LEFT_TRAP));
            this.driverOne.b().whileTrue(this.swerve.pathfind(Constants.SwerveConstants.RED_CENTER_TRAP));
            this.driverOne.y().whileTrue(this.swerve.pathfind(Constants.SwerveConstants.RED_RIGHT_TRAP));
        }

        this.driverTwo.povLeft().onTrue(new InstantCommand(() -> {

            this.regression = false;
            this.arm.setAngle(Constants.ArmConstants.SUBWOOFER_ANGLE);
            this.shooter.setSpeed(Constants.ShooterConstants.SPEAKER_SPEED);
        }));

        this.driverTwo.povDown().onTrue(new InstantCommand(() -> {

            this.regression = false;
            this.arm.setAngle(Constants.ArmConstants.LAUNCHPAD_ANGLE);
            this.shooter.setSpeed(Constants.ShooterConstants.SPEAKER_SPEED);
        }));

        this.driverTwo.povUp().onTrue(new InstantCommand(() -> {

            this.regression = false;
            this.arm.setAngle(Constants.ArmConstants.AMP_ANGLE);
            this.shooter.setSpeed(Constants.ShooterConstants.AMP_SPEED);
        }));


        this.driverTwo.povRight().onTrue(new InstantCommand(() -> {

            this.regression = false;
            this.arm.setAngle(Constants.ArmConstants.FERRY_ANGLE);
            this.shooter.setSpeed(Constants.ShooterConstants.FERRY_SPEED);
        }));

        this.driverTwo.x().onTrue(new InstantCommand(() -> this.transferObjective(Objective.SPEAKER)));
        this.driverTwo.a().onTrue(new InstantCommand(() -> this.transferObjective(Objective.AMP)));
        this.driverTwo.b().onTrue(new InstantCommand(() -> this.transferObjective(Objective.TRAP)));
        this.driverTwo.y().onTrue(new InstantCommand(() -> this.transferObjective(Objective.STAGE)));

        this.driverTwo.leftBumper().onTrue(new InstantCommand(() -> {

            Class<? extends Command> intakeState = this.intakeStateMachine.getCurrentState();
            Class<? extends Command> indexerState = this.indexerStateMachine.getCurrentState();

            if (intakeState != OutakeNote.class && indexerState != ReverseNote.class) {

                this.intakeStateMachine.activateState(OutakeNote.class);
                this.indexerStateMachine.activateState(ReverseNote.class);
            } else {

                this.intakeStateMachine.activateState(IdleIntake.class);
                this.indexerStateMachine.activateState(IdleIndexer.class);
            }
        }));

        this.driverTwo.rightBumper().onTrue(new InstantCommand(() -> {

            Class<? extends Command> intakeState = this.intakeStateMachine.getCurrentState();
            Class<? extends Command> indexerState = this.indexerStateMachine.getCurrentState();

            if (intakeState != IntakeNote.class) { this.intakeStateMachine.activateState(IntakeNote.class); }
            else { this.intakeStateMachine.activateState(IdleIntake.class); }

            if (this.objective == Objective.SPEAKER) {

                if (indexerState != IndexSpeakerNote.class) { this.indexerStateMachine.activateState(IndexSpeakerNote.class); }
                else { this.indexerStateMachine.activateState(IdleIndexer.class); }
            }

            if (this.objective == Objective.AMP) {

                if (indexerState != IndexAmpNote.class) { this.indexerStateMachine.activateState(IndexAmpNote.class); }
                else { this.indexerStateMachine.activateState(IdleIndexer.class); }
            }

            if (this.objective == Objective.TRAP) {

                if (indexerState != FeedNote.class) { this.indexerStateMachine.activateState(FeedNote.class); }
                else { this.indexerStateMachine.activateState(IdleIndexer.class); }
            }
        }));

        this.driverTwo.leftTrigger().onTrue(new InstantCommand(() -> {

            this.elevatorStateMachine.activateState(LockElevator.class);
            this.armStateMachine.activateState(LockArm.class);
            this.shooterStateMachine.activateState(IdleShooter.class);
        }));

        this.driverTwo.rightTrigger().onTrue(new InstantCommand(() -> {

            if (this.objective == Objective.SPEAKER) {

                this.armStateMachine.activateState(PivotArm.class);
                this.shooterStateMachine.activateState(ShootNote.class);
            }

            if (this.objective == Objective.AMP) {

                this.elevatorStateMachine.activateState(RaiseElevator.class);
            }

            if (this.objective == Objective.TRAP) {

                this.armStateMachine.activateState(PivotArm.class);
                this.shooterStateMachine.activateState(ShootNote.class);
            }
        }));

        this.driverTwo.rightTrigger().onFalse(new InstantCommand(() -> {

            if (this.objective == Objective.SPEAKER) {

                this.indexerStateMachine.activateState(FeedNote.class);
            }

            if (this.objective == Objective.AMP) {

                this.indexerStateMachine.activateState(DropNote.class);
            }

            if (this.objective == Objective.TRAP) {

                this.indexerStateMachine.activateState(FeedNote.class);
            }
        }));

        this.driverTwo.start().onTrue(new InstantCommand(() -> {

            this.intakeStateMachine.activateState(IdleIntake.class);
            this.elevatorStateMachine.activateState(LockElevator.class);
            this.indexerStateMachine.activateState(IdleIndexer.class);
            this.armStateMachine.activateState(LockArm.class);
            this.shooterStateMachine.activateState(IdleShooter.class);

            this.regression = true;
        }));
    }

    public void calculateRegression () {

        if (this.regression && this.objective == Objective.SPEAKER) {

            this.arm.calculateAngle(this.swerve.getSpeakerDistance());
            this.shooter.setSpeed(Constants.ShooterConstants.SPEAKER_SPEED);
        } else if (this.objective == Objective.TRAP) {

            this.arm.setAngle(Constants.ArmConstants.TRAP_ANGLE);
            this.shooter.setSpeed(Constants.ShooterConstants.TRAP_SPEED);
        }
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
        Class<? extends Command> indexerState = this.indexerStateMachine.getCurrentState();

        boolean intakeFinished = this.intakeStateMachine.currentCommandFinished();
        boolean indexerFinished = this.indexerStateMachine.currentCommandFinished();

        if (intakeState != IdleIntake.class && intakeFinished) {

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
            this.armStateMachine.activateState(LockArm.class);
            this.shooterStateMachine.activateState(IdleShooter.class);
            
            this.regression = true;
        }

        if (indexerState == DropNote.class && indexerFinished) {

            this.elevatorStateMachine.activateState(LockElevator.class);
            this.indexerStateMachine.activateState(IdleIndexer.class);
        }

        double elevatorUp = MathUtil.applyDeadband(this.driverOne.getLeftTriggerAxis(), Constants.SwerveConstants.TRANSLATION_DEADBAND);
        double elevatorDown = MathUtil.applyDeadband(this.driverOne.getRightTriggerAxis(), Constants.SwerveConstants.TRANSLATION_DEADBAND);
        this.elevator.setSpeed(elevatorUp - elevatorDown);
    }

    public void runAutoStateMachines () {

        Class<? extends Command> intakeState = this.intakeStateMachine.getCurrentState();
        Class<? extends Command> indexerState = this.indexerStateMachine.getCurrentState();
        Class<? extends Command> armState = this.armStateMachine.getCurrentState();
        Class<? extends Command> shooterState = this.shooterStateMachine.getCurrentState();

        boolean intakeFinished = this.intakeStateMachine.currentCommandFinished();
        boolean indexerFinished = this.indexerStateMachine.currentCommandFinished();

        if (intakeState == IntakeNote.class && intakeFinished) { this.intakeStateMachine.activateState(IdleIntake.class); }
        if (indexerState == IndexSpeakerNote.class && indexerFinished) { this.indexerStateMachine.activateState(HoldSpeakerNote.class); }

        if (indexerState == HoldSpeakerNote.class && armState == PivotArm.class && shooterState == ShootNote.class) {

            if (this.arm.atPosition() && this.shooter.atSpeed()) { 
                
                this.indexerStateMachine.activateState(FeedNote.class); 
            }
        }
    }

    public void transferObjective (Objective newObjective) {

        Class<? extends Command> elevatorState = this.elevatorStateMachine.getCurrentState();
        Class<? extends Command> indexerState = this.indexerStateMachine.getCurrentState();

        if (this.objective == Objective.AMP && (newObjective == Objective.SPEAKER || newObjective == Objective.TRAP)) {

            if (indexerState == IndexAmpNote.class) { this.indexerStateMachine.activateState(IndexSpeakerNote.class); }
            if (indexerState == HoldAmpNote.class && elevatorState == LockElevator.class) { this.indexerStateMachine.activateState(IndexSpeakerNote.class); }
        } else if (this.objective != Objective.STAGE && newObjective == Objective.STAGE) {

            this.elevatorStateMachine.activateState(ManualElevator.class);
            this.armStateMachine.activateState(LockArm.class);
        } else if (this.objective == Objective.STAGE && newObjective != Objective.STAGE) {

            this.elevatorStateMachine.activateState(LockElevator.class);
        }

        this.objective = newObjective;
    }

    public void runLEDs () {

        Class<? extends Command> elevatorState = this.elevatorStateMachine.getCurrentState();
        Class<? extends Command> indexerState = this.indexerStateMachine.getCurrentState();
        Class<? extends Command> armState = this.armStateMachine.getCurrentState();
        Class<? extends Command> shooterState = this.shooterStateMachine.getCurrentState();

        if (!DriverStation.isTeleopEnabled()) {

            this.leds.setPattern(Rainbow.class);
            return;
        }

        if (this.objective == Objective.SPEAKER) { 
            
            if (indexerState == IndexSpeakerNote.class) { 
                
                if (!Sensors.getIndexerStartBeam() || !Sensors.getIndexerEndBeam()) {

                    this.leds.setPattern(NoteBlink.class);
                } else {

                    this.leds.setPattern(BlueBlink.class); 
                }
            } else if (indexerState == HoldSpeakerNote.class) {

                if (armState == PivotArm.class && shooterState == ShootNote.class) {

                    if (this.arm.atPosition() && this.shooter.atSpeed()) {

                        this.leds.setPattern(Note.class);
                    } else {

                        this.leds.setPattern(NoteBlink.class);
                    }
                } else {

                    this.leds.setPattern(Note.class);
                }
            } else {

                this.leds.setPattern(Blue.class); 
            }
        } else if (this.objective == Objective.AMP) { 
            
            if (indexerState == IndexAmpNote.class) {

                if (!Sensors.getIndexerStartBeam()) {

                    this.leds.setPattern(NoteBlink.class);
                } else {

                    this.leds.setPattern(GreenBlink.class);
                }
            } else if (indexerState == HoldAmpNote.class) {

                if (elevatorState == RaiseElevator.class) {

                    if (this.elevator.atHeight()) {

                        this.leds.setPattern(Note.class);
                    } else {

                        this.leds.setPattern(NoteBlink.class);
                    }
                } else {

                    this.leds.setPattern(Note.class);
                }
            } else {

                this.leds.setPattern(Green.class); 
            }
        } else if (this.objective == Objective.STAGE) { 
            
            if (elevatorState != ManualElevator.class) {

                this.leds.setPattern(YellowBlink.class);
            } else {
                
                this.leds.setPattern(Yellow.class);
            }
        }
    }

    public void initializePathPlanner () {

        NamedCommands.registerCommand("Shoot", new AutoScoreNote(this.intakeStateMachine, this.indexerStateMachine, this.armStateMachine, this.shooterStateMachine));
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
                new ReplanningConfig()
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
