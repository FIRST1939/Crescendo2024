// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.arm.LockArm;
import frc.robot.commands.elevator.LockElevator;
import frc.robot.commands.indexer.HoldSpeakerNote;
import frc.robot.commands.indexer.IdleIndexer;
import frc.robot.commands.intake.IdleIntake;
import frc.robot.commands.shooter.IdleShooter;
import frc.robot.util.Alerts;
import frc.robot.util.BuildConstants;
import frc.robot.util.Constants;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

	private RobotContainer robotContainer;
	private boolean autoInitialized = false;
	private Command autonomousCommand;
	private Timer disabledTimer;
	
	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	@Override
	public void robotInit () {

		if (BuildConstants.DIRTY == 1) { Alerts.versionControl.set(true); }

		this.robotContainer = new RobotContainer();
		this.disabledTimer = new Timer();

		Shuffleboard.getTab("Autonomous").add("Command Scheduler", CommandScheduler.getInstance());
		CameraServer.startAutomaticCapture();
	}

	@Override
	public void robotPeriodic () {

		CommandScheduler.getInstance().run();
		this.robotContainer.runLEDs();
	}

	@Override
	public void autonomousInit () {

		this.robotContainer.setIdleModes(
			Constants.SwerveConstants.ENABLED_IDLE_BEHAVIOR,
			Constants.IntakeConstants.ENABLED_IDLE_BEHAVIOR,
			Constants.ElevatorConstants.ENABLED_IDLE_BEHAVIOR,
			Constants.IndexerConstants.ENABLED_IDLE_BEHAVIOR,
			Constants.ArmConstants.ENABLED_IDLE_BEHAVIOR,
			Constants.ShooterConstants.ENABLED_IDLE_BEHAVIOR
		);

		this.robotContainer.initializeStateMachines(
			IdleIntake.class,
			LockElevator.class,
			HoldSpeakerNote.class,
			LockArm.class,
			IdleShooter.class
		);

		this.autoInitialized = true;
		Constants.SwerveConstants.REGRESSION = false;

		this.autonomousCommand = this.robotContainer.getAutonomousCommand();
		this.autonomousCommand.schedule();
	}

	@Override
	public void autonomousPeriodic () {

		this.robotContainer.runAutoStateMachines();
	}

	@Override
	public void teleopInit () {

		if (this.autonomousCommand != null) { this.autonomousCommand.cancel(); }

		if (!this.autoInitialized) {
			this.robotContainer.setIdleModes(
				Constants.SwerveConstants.ENABLED_IDLE_BEHAVIOR,
				Constants.IntakeConstants.ENABLED_IDLE_BEHAVIOR,
				Constants.ElevatorConstants.ENABLED_IDLE_BEHAVIOR,
				Constants.IndexerConstants.ENABLED_IDLE_BEHAVIOR,
				Constants.ArmConstants.ENABLED_IDLE_BEHAVIOR,
				Constants.ShooterConstants.ENABLED_IDLE_BEHAVIOR
			);

			this.robotContainer.initializeStateMachines(
				IdleIntake.class,
				LockElevator.class,
				IdleIndexer.class,
				LockArm.class,
				IdleShooter.class
			);
		}

		Constants.SwerveConstants.REGRESSION = true;
	}

	@Override
	public void teleopPeriodic () {

		this.robotContainer.runStateMachines();
	}

	@Override
	public void disabledInit () {

		this.disabledTimer.reset();
		this.disabledTimer.start();
	}

	@Override
	public void disabledPeriodic () {

		if (this.disabledTimer.get() >= Constants.SwerveConstants.LOCK_TIME) {

			this.robotContainer.setIdleModes(
				Constants.SwerveConstants.DISABLED_IDLE_BEHAVIOR,
				Constants.IntakeConstants.DISABLED_IDLE_BEHAVIOR,
				Constants.ElevatorConstants.DISABLED_IDLE_BEHAVIOR,
				Constants.IndexerConstants.DISABLED_IDLE_BEHAVIOR,
				Constants.ArmConstants.DISABLED_IDLE_BEHAVIOR,
				Constants.ShooterConstants.DISABLED_IDLE_BEHAVIOR
			);

			this.disabledTimer.stop();
			this.disabledTimer.reset();
		}

		if (RobotController.getBatteryVoltage() < Constants.RobotConstants.MINIMUM_BATTERY_VOLTAGE) { Alerts.lowBattery.set(true); }
		else { Alerts.lowBattery.set(false); }
	}
	
	@Override
	public void testInit () {}

	@Override
	public void testPeriodic () {}

	@Override
	public void simulationInit () {}

	@Override
	public void simulationPeriodic () {}
}
