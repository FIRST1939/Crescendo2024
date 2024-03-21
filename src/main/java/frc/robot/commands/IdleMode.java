package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Constants.IdleBehavior;

public class IdleMode extends InstantCommand {
    
    private Swerve swerve;
    private IdleBehavior swerveIdle;

    private Intake intake;
    private IdleBehavior intakeIdle;

    private Elevator elevator;
    private IdleBehavior idleElevator;
    
    private Indexer indexer;
    private IdleBehavior indexerIdle;

    private Arm arm;
    private IdleBehavior armIdle;

    private Shooter shooter;
    private IdleBehavior shooterIdle;

    public IdleMode (Swerve swerve, IdleBehavior swerveIdle, Intake intake, IdleBehavior intakeIdle, Elevator elevator, IdleBehavior elevatorIdle, Indexer indexer, IdleBehavior indexerIdle, Arm arm, IdleBehavior armIdle, Shooter shooter, IdleBehavior shooterIdle) {

        this.swerve = swerve;
        this.swerveIdle = swerveIdle;

        this.intake = intake;
        this.intakeIdle = intakeIdle;

        this.elevator = elevator;
        this.idleElevator = elevatorIdle;

        this.indexer = indexer;
        this.indexerIdle = indexerIdle;

        this.arm = arm;
        this.armIdle = armIdle;

        this.shooter = shooter;
        this.shooterIdle = shooterIdle;
    }

    @Override
    public void initialize () {

        this.swerve.setIdleBehavior(this.swerveIdle);
        this.intake.setIdleBehavior(this.intakeIdle);
        this.elevator.setIdleBehavior(this.idleElevator);
        this.indexer.setIdleBehavior(this.indexerIdle);
        this.arm.setIdleBehavior(this.armIdle);
        this.shooter.setIdleBehavior(this.shooterIdle);
    }

    @Override
    public boolean runsWhenDisabled () { return true; }

    @Override
    public boolean isFinished () { return true; }
}
