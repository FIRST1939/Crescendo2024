// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
	}

	@Override
	public void robotPeriodic () {

		CommandScheduler.getInstance().run();
	}

	@Override
	public void autonomousInit () {

		this.autonomousCommand = this.robotContainer.getAutonomousCommand();
		this.autonomousCommand.schedule();
	}

	@Override
	public void autonomousPeriodic () {}

	@Override
	public void teleopInit () {

		if (this.autonomousCommand != null) { this.autonomousCommand.cancel(); }
		this.robotContainer.setBrakeMode(true);
	}

	@Override
	public void teleopPeriodic () {}

	@Override
	public void disabledInit () {

		this.disabledTimer.reset();
		this.disabledTimer.start();
	}

	@Override
	public void disabledPeriodic () {

		if (this.disabledTimer.get() >= Constants.SwerveConstants.LOCK_TIME) {

			this.robotContainer.setBrakeMode(false);
		}
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
