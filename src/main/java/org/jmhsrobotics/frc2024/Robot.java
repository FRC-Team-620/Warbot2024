// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.jmhsrobotics.frc2024;

import java.util.HashMap;
import java.util.Map;

import org.jmhsrobotics.frc2024.utils.GameObjectSim;
import org.jmhsrobotics.warcore.util.BuildDataLogger;
import org.littletonrobotics.urcl.URCL;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
	private Command autonomousCommand;

	private RobotContainer m_robotContainer;
	public static GameObjectSim objSim;

	@Override
	public void robotInit() {
		if (Robot.isSimulation()) {
			objSim = new GameObjectSim();
			objSim.preload();
		}

		m_robotContainer = new RobotContainer();
		m_robotContainer.getDriveSubsystem().zeroHeading();
		setupLogs();
	}

	public void setupLogs() {
		if (Robot.isSimulation()) {
			// Filesystem.getOperatingDirectory().getAbsolutePath().
			DataLogManager.start(Filesystem.getOperatingDirectory().getAbsolutePath() + "/logs");
		} else {
			DataLogManager.start();
		}
		DataLogManager.logNetworkTables(true);
		DriverStation.startDataLog(DataLogManager.getLog());
		BuildDataLogger.LogToNetworkTables(BuildConstants.class);
		BuildDataLogger.LogToWpiLib(DataLogManager.getLog(), BuildConstants.class);
		boolean fileOnly = false;
		boolean lazyLogging = true;
		URCL.start(getCanIDMap());
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {
	}

	@Override
	public void disabledExit() {
	}

	@Override
	public void autonomousInit() {
		if (Robot.isSimulation()) {
			Robot.objSim.reset();
			Robot.objSim.preload();
		}
		m_robotContainer.getDriveSubsystem().zeroHeading();
		autonomousCommand = m_robotContainer.getAutonomousCommand();
		m_robotContainer.getArmSubsystem().setBreak();

		if (autonomousCommand != null) {
			autonomousCommand.schedule();
		}

	}

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void teleopInit() {
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
		m_robotContainer.getArmSubsystem().setBreak();
		m_robotContainer.configureTeam();
	}

	@Override
	public void teleopPeriodic() {

	}

	@Override
	public void teleopExit() {
	}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void testPeriodic() {
	}

	@Override
	public void testExit() {
	}

	@Override
	public void simulationInit() {
		DriverStationSim.setDsAttached(true);
		DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
		DriverStationSim.setEnabled(true);

		DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
	}

	private Map<Integer, String> getCanIDMap() {
		var map = new HashMap<Integer, String>();

		map.put(Constants.SwerveConstants.kFrontLeftDrivingCanId, "FrontLeftDriving");
		map.put(Constants.SwerveConstants.kFrontLeftTurningCanId, "FrontLeftTurningCan");
		map.put(Constants.SwerveConstants.kFrontRightDrivingCanId, "FrontRightDriving");
		map.put(Constants.SwerveConstants.kFrontRightTurningCanId, "FrontRightTurning");
		map.put(Constants.SwerveConstants.kRearLeftDrivingCanId, "RearLeftDriving");
		map.put(Constants.SwerveConstants.kRearLeftTurningCanId, "RearLeftTurning");
		map.put(Constants.SwerveConstants.kRearRightDrivingCanId, "RearRightDriving");
		map.put(Constants.SwerveConstants.kRearRightTurningCanId, "RearRightTurning");

		map.put(Constants.CAN.kArmPivotFollowerID, "ArmPivotFollower");
		map.put(Constants.CAN.kArmPivotRightID, "ArmPivotRight");

		map.put(Constants.CAN.kIntakeId, "Intake");
		map.put(Constants.CAN.kShooterBottomId, "ShooterBottom");
		map.put(Constants.CAN.kShooterTopId, "ShooterTop");

		map.put(Constants.CAN.kLeftClimberID, "LeftClimber");
		map.put(Constants.CAN.kRightClimberID, "RightClimber");
		return map;
	}
}
