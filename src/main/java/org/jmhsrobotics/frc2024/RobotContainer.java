// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.jmhsrobotics.frc2024;

import org.jmhsrobotics.frc2024.ComboCommands.AmpHelper;
import org.jmhsrobotics.frc2024.ComboCommands.ComboIntakeArmCommand;
import org.jmhsrobotics.frc2024.autoCommands.AutoAmpShotCommand;
import org.jmhsrobotics.frc2024.autoCommands.FireCommand;
import org.jmhsrobotics.frc2024.autoCommands.TurnAndShootCommand;
import org.jmhsrobotics.frc2024.controlBoard.CompControl;
import org.jmhsrobotics.frc2024.controlBoard.ControlBoard;
import org.jmhsrobotics.frc2024.controlBoard.SingleControl;
import org.jmhsrobotics.frc2024.controlBoard.SwitchableControlBoard;
import org.jmhsrobotics.frc2024.subsystems.LED.LEDSubsystem;
import org.jmhsrobotics.frc2024.subsystems.LED.commands.RainbowLEDCommand;
import org.jmhsrobotics.frc2024.subsystems.LED.commands.setBlueLEDCommand;
import org.jmhsrobotics.frc2024.subsystems.LED.commands.setRedLEDCommand;
import org.jmhsrobotics.frc2024.subsystems.LED.commands.setYellowLEDCommand;
import org.jmhsrobotics.frc2024.subsystems.arm.ArmPIDSubsystem;
import org.jmhsrobotics.frc2024.subsystems.arm.commands.ArmVision;
import org.jmhsrobotics.frc2024.subsystems.arm.commands.CommandArm;
import org.jmhsrobotics.frc2024.subsystems.arm.commands.PrepareShot;
import org.jmhsrobotics.frc2024.subsystems.arm.commands.ToggleBakes;
import org.jmhsrobotics.frc2024.subsystems.climber.ClimberSubsystem;
import org.jmhsrobotics.frc2024.subsystems.drive.DriveSubsystem;
import org.jmhsrobotics.frc2024.subsystems.drive.commands.DriveCommand;
import org.jmhsrobotics.frc2024.subsystems.drive.commands.LockSpeaker;
import org.jmhsrobotics.frc2024.subsystems.drive.commands.auto.DriveTimeCommand;
import org.jmhsrobotics.frc2024.subsystems.drive.commands.auto.NoteHunterCommand;
import org.jmhsrobotics.frc2024.subsystems.intake.IntakeSubsystem;
import org.jmhsrobotics.frc2024.subsystems.intake.commands.AmpShotCommand;
import org.jmhsrobotics.frc2024.subsystems.intake.commands.AutoIntakeCommand;
import org.jmhsrobotics.frc2024.subsystems.intake.commands.DefaultIntakeCommand;
import org.jmhsrobotics.frc2024.subsystems.intake.commands.ExtakeCommand;
import org.jmhsrobotics.frc2024.subsystems.intake.commands.IntakeCommand;
import org.jmhsrobotics.frc2024.subsystems.intake.commands.IntakeFireCommand;
import org.jmhsrobotics.frc2024.subsystems.shooter.ShooterSubsystem;
import org.jmhsrobotics.frc2024.subsystems.shooter.commands.ShooterAutoCommand;
import org.jmhsrobotics.frc2024.subsystems.vision.VisionSubsystem;
import org.jmhsrobotics.frc2024.utils.RumbleTimeCommand;
import org.jmhsrobotics.frc2024.utils.newcmd.NFireAmp;
import org.jmhsrobotics.frc2024.utils.newcmd.NFloorIntake;
import org.jmhsrobotics.frc2024.utils.newcmd.NSpinupAndShoot;
import org.jmhsrobotics.frc2024.utils.newcmd.NSpinupNoStop;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import monologue.Logged;

public class RobotContainer implements Logged {

	public final ControlBoard control;
	// Subsystems
	private final DriveSubsystem driveSubsystem = new DriveSubsystem();

	private final VisionSubsystem visionSubsystem = new VisionSubsystem(this.driveSubsystem);

	private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
	private final LEDSubsystem ledSubsystem = new LEDSubsystem();
	private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

	private final ArmPIDSubsystem armSubsystem = new ArmPIDSubsystem();

	private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

	private final SendableChooser<Command> autoChooser;

	// private final DefaultIntakeCommand defaultIntakeCommand = new
	// DefaultIntakeCommand(intakeSubsystem,
	// shooterSubsystem);
	// private final DefaultIntakeCommand defaultIntakeCommand = new
	// DefaultIntakeCommand(intakeSubsystem,
	// shooterSubsystem);

	private final DriveCommand driveCommand;

	public RobotContainer() {
		SwitchableControlBoard swboard = new SwitchableControlBoard(new CompControl());
		if (Robot.isSimulation()) { // Switch to single control in sim
			swboard.setControlBoard(new SingleControl());
		}
		// swboard.setControlBoard(new CompControl());

		this.control = swboard;
		this.driveCommand = new DriveCommand(this.driveSubsystem, visionSubsystem, this.control);
		this.driveSubsystem
				.setDefaultCommand(new DriveCommand(this.driveSubsystem, this.visionSubsystem, this.control));

		this.intakeSubsystem.setDefaultCommand(new DefaultIntakeCommand(intakeSubsystem, shooterSubsystem));

		this.ledSubsystem.setDefaultCommand(new RainbowLEDCommand(this.ledSubsystem));

		configureSmartDashboard();
		configureDriverFeedback();

		configureBindings();
		// Named commands must be added before building the chooser.
		configurePathPlanner();
		autoChooser = AutoBuilder.buildAutoChooser();
		autoChooser.setDefaultOption("BaseLineAuto", new DriveTimeCommand(2.2, 0.3, driveSubsystem));

		var preLoadOnePiece = Commands.sequence(
				Commands.race(new CommandArm(this.armSubsystem, Constants.ArmSetpoint.SHOOT.value),
						new NSpinupNoStop(this.shooterSubsystem, 4000)),
				new NSpinupAndShoot(this.shooterSubsystem, this.intakeSubsystem, 4000));
		autoChooser.addOption("preLoadOnePiece", preLoadOnePiece);

		SmartDashboard.putData("Auto Chooser", autoChooser);
		SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());
		// Commands to test
		SmartDashboard.putData("Arm Preset Shoot",
				new CommandArm(this.armSubsystem, Constants.ArmSetpoint.SHOOT.value));
		SmartDashboard.putData("Intake Floor", new NFloorIntake(armSubsystem, intakeSubsystem));
		SmartDashboard.putData("Fire in Amp", new NFireAmp(this.shooterSubsystem, this.intakeSubsystem));
		SmartDashboard.putData("Spinup and Shoot", new NSpinupAndShoot(shooterSubsystem, intakeSubsystem, 4000));
		SmartDashboard.putData("Spinup no Stop", new NSpinupNoStop(shooterSubsystem, 4000));
		SmartDashboard.putData("Aim Arm Vision",
				new ArmVision(armSubsystem, visionSubsystem, driveSubsystem).until(armSubsystem::atGoal)); // TODO:
																											// Handle
																											// End
		// // condition
		// SmartDashboard.putData("RedLED", new setRedLEDCommand(ledSubsystem,
		// intakeSubsystem));
		SmartDashboard.putData("ArmVision", new ArmVision(armSubsystem, visionSubsystem, driveSubsystem));
		SmartDashboard.putData("Lock Speaker", new LockSpeaker(driveSubsystem, visionSubsystem));
		SmartDashboard.putData("NoteHunter", new NoteHunterCommand(visionSubsystem, driveSubsystem));

	}

	private void configureDriverFeedback() {
		new Trigger(intakeSubsystem::hasNote)
				.onTrue(new ParallelCommandGroup(new RumbleTimeCommand(control, RumbleType.kLeftRumble, 1, 1),
						new setBlueLEDCommand(ledSubsystem, this.intakeSubsystem)));

		// new Trigger(this.defaultIntakeCommand::isScheduled)
		// .onTrue(new FlashingLEDCommand(ledSubsystem, intakeSubsystem));
		// new Trigger(intakeSubsystem::isIntaking).onTrue(new
		// FlashingLEDCommand(ledSubsystem, intakeSubsystem));

		// new Trigger(intakeSubsystem.getCurrentCommand()==new
		// DefaultIntakeCommand(this.intakeSubsystem, this.shooterSubsystem)).onTrue(new
		// FlashingLEDCommand(ledSubsystem, intakeSubsystem));
		// new Trigger(this.defaultIntakeCommand::isScheduled)
		// .onTrue(new FlashingLEDCommand(ledSubsystem, intakeSubsystem));

		new Trigger(intakeSubsystem::noteTooHigh).onTrue(new setRedLEDCommand(ledSubsystem, intakeSubsystem));

		new Trigger(shooterSubsystem::atGoal)
				.whileTrue(new RumbleTimeCommand(this.control, RumbleType.kRightRumble, 0.2, 1));

		new Trigger(shooterSubsystem::atGoal).onTrue(new setYellowLEDCommand(ledSubsystem, shooterSubsystem));
	}

	private void configurePathPlanner() {
		// Add path planner auto chooser.
		var preLoadOnePiece = Commands.sequence(
				Commands.race(new CommandArm(this.armSubsystem, Constants.ArmSetpoint.SHOOT.value),
						new NSpinupNoStop(this.shooterSubsystem, 5000)),
				new NSpinupAndShoot(this.shooterSubsystem, this.intakeSubsystem, 5000));

		AutoBuilder.configureHolonomic(driveSubsystem::getPose, driveSubsystem::resetOdometry,
				driveSubsystem::getChassisSpeeds, driveSubsystem::drive,
				new HolonomicPathFollowerConfig(new PIDConstants(5, 0, 0), new PIDConstants(1.5, 0, 0),
						Constants.SwerveConstants.kMaxSpeedMetersPerSecond, .5, new ReplanningConfig()),
				this::getAllianceFlipState, driveSubsystem);

		NamedCommands.registerCommand("ArmAmp", new CommandArm(this.armSubsystem, Constants.ArmSetpoint.AMP.value));
		NamedCommands.registerCommand("ArmShoot", new CommandArm(this.armSubsystem, Constants.ArmSetpoint.SHOOT.value));
		NamedCommands.registerCommand("Extake", new ExtakeCommand(this.intakeSubsystem, 1).withTimeout(5));
		NamedCommands.registerCommand("TurnAndShoot", new TurnAndShootCommand(this.visionSubsystem, this.driveSubsystem,
				this.armSubsystem, this.shooterSubsystem, this.intakeSubsystem));
		NamedCommands.registerCommand("Intake",
				new IntakeCommand(1, this.intakeSubsystem, this.shooterSubsystem).withTimeout(0.25));
		NamedCommands.registerCommand("AutoIntake",
				new AutoIntakeCommand(1, this.intakeSubsystem, this.shooterSubsystem));

		// Move Arm to Pickup position
		NamedCommands.registerCommand("ArmPickup",
				new CommandArm(this.armSubsystem, Constants.ArmSetpoint.PICKUP.value));
		NamedCommands.registerCommand("Fire", new FireCommand(this.intakeSubsystem, this.shooterSubsystem));
		NamedCommands.registerCommand("PrepareShot",
				new PrepareShot(this.driveSubsystem, this.armSubsystem, this.shooterSubsystem, this.visionSubsystem)
						.withTimeout(1));
		NamedCommands.registerCommand("AmpScore",
				new AmpHelper(this.armSubsystem, this.shooterSubsystem, this.intakeSubsystem));

		// New Commands
		NamedCommands.registerCommand("Arm Preset Shoot",
				new CommandArm(this.armSubsystem, Constants.ArmSetpoint.SHOOT.value));
		NamedCommands.registerCommand("Intake Floor", new NFloorIntake(armSubsystem, intakeSubsystem).withTimeout(4));
		NamedCommands.registerCommand("Fire in Amp", new NFireAmp(this.shooterSubsystem, this.intakeSubsystem));
		NamedCommands.registerCommand("Spinup and Shoot", new NSpinupAndShoot(shooterSubsystem, intakeSubsystem, 5000));
		NamedCommands.registerCommand("Spinup no Stop", new NSpinupNoStop(shooterSubsystem, 5000));
		NamedCommands.registerCommand("Aim Arm Vision", new ArmVision(armSubsystem, visionSubsystem, driveSubsystem)); // TODO:
																														// Handle
																														// End
		// temp in order to adjust note in intake
		NamedCommands.registerCommand("IntakeAdjust", new IntakeFireCommand(1, this.intakeSubsystem).withTimeout(.17));
		// condition
		NamedCommands.registerCommand("Lock Speaker", new LockSpeaker(driveSubsystem, visionSubsystem));
		NamedCommands.registerCommand("ComboIntake",
				new ComboIntakeArmCommand(this.armSubsystem, this.shooterSubsystem, this.intakeSubsystem)
						.withTimeout(6));
		// NamedCommands.registerCommand("AmpShoot", new AmpShotCommand(intakeSubsystem,
		// shooterSubsystem).withTimeout(1));
		NamedCommands.registerCommand("AmpShoot", new AutoAmpShotCommand(this.intakeSubsystem, this.shooterSubsystem));
		NamedCommands.registerCommand("One Piece Preload Shoot", preLoadOnePiece);
	}

	// TODO: fix this later to flip correctly based on side color
	// TODO: Check parity of this
	private boolean getAllianceFlipState() {
		return DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() == Alliance.Red : false;
	}

	private void configureBindings() {
		// this.control.Rumble();
		new Trigger(RobotController::getUserButton).onTrue(new ToggleBakes(armSubsystem));
		/* Arm Controls */

		// Move Arm to Amp position
		this.control.presetHigh().onTrue(new CommandArm(this.armSubsystem, Constants.ArmSetpoint.AMP.value));
		// Move Arm to Preset Shoot position
		this.control.presetMid().onTrue(new CommandArm(this.armSubsystem, Constants.ArmSetpoint.SHOOT.value));
		// this.control.presetLow().onTrue(new CommandArm(this.armSubsystem,
		// Constants.ArmSetpoint.PICKUP.value));
		this.control.presetLow().whileTrue(new ComboIntakeArmCommand(armSubsystem, shooterSubsystem, intakeSubsystem));
		this.control.presetLow().onFalse(new CommandArm(this.armSubsystem, Constants.ArmSetpoint.SHOOT.value));

		/* Intake Controls */
		this.control.intakeInput().whileTrue(new IntakeFireCommand(1, this.intakeSubsystem));
		this.control.extakeInput().whileTrue(new IntakeCommand(-1, this.intakeSubsystem, this.shooterSubsystem));

		/* Shooter Controls */
		this.control.shooterInput().whileTrue(new ShooterAutoCommand(this.shooterSubsystem, 5000));
		this.control.ampShooterInput().whileTrue(new AmpShotCommand(intakeSubsystem, shooterSubsystem));

		// temp climber controls
		// this.control.climberRetract().onTrue(new ClimbCommand(this.climberSubsystem,
		// -10.919127));
		// this.control.climberExtend().onTrue(new ClimbCommand(this.climberSubsystem,
		// 0));
		this.control.climberRetract().whileTrue(new InstantCommand(this.climberSubsystem::climberRetract));
		this.control.climberRetract().onFalse(new InstantCommand(this.climberSubsystem::climberStop));
		this.control.climberExtend().whileTrue(new InstantCommand(this.climberSubsystem::climberExtend));
		this.control.climberExtend().onFalse(new InstantCommand(this.climberSubsystem::climberStop));

	}

	public ArmPIDSubsystem getArmSubsystem() {
		return armSubsystem;
	}

	public void configureSmartDashboard() {
		SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());

		SmartDashboard.putBoolean("LockAtGoal", this.driveCommand.lockAtGoal());
		SmartDashboard.putNumber("MatchTime", DriverStation.getMatchTime());
		SmartDashboard.putNumber("MatchNumber", DriverStation.getMatchNumber());
		// SmartDashboard.putData("lock speaker", new LockSpeaker(this.driveSubsystem,
		// this.visionSubsystem));
		// SmartDashboard.putData("AutoIntakeCommand",
		// new AutoIntakeCommand(1, this.intakeSubsystem, this.shooterSubsystem));

		// SmartDashboard.putData("AutoShooterCommand", new
		// ShooterAutoCommand(this.shooterSubsystem, 1));

		// SmartDashboard.putData("FireCommand", new FireCommand(this.intakeSubsystem,
		// this.shooterSubsystem));

		// SmartDashboard.putData("AmpHelper",
		// new AmpHelper(this.armSubsystem, this.shooterSubsystem,
		// this.intakeSubsystem));

		// SmartDashboard.putData("LockAprilTagCommand", new LockAprilTag(7,
		// this.driveSubsystem, this.visionSubsystem));
	}
	public void configureTeam() {
		this.control.AprilLockOn()
				.whileTrue(Commands.repeatingSequence(new ArmVision(armSubsystem, visionSubsystem, driveSubsystem)));
		this.control.AprilLockOn().whileTrue(new ShooterAutoCommand(shooterSubsystem, 5000));
		// if (getAllianceFlipState()) {
		// this.control.AprilLockOn()
		// .whileTrue(Commands.repeatingSequence(new PrepareShot(driveSubsystem,
		// armSubsystem, shooterSubsystem, visionSubsystem)));
		// } else {
		// this.control.AprilLockOn()
		// .whileTrue(Commands.repeatingSequence(new PrepareShot(driveSubsystem,
		// armSubsystem, shooterSubsystem, visionSubsystem)));
		// }
	}

	public Command getAutonomousCommand() {
		Command picked = autoChooser.getSelected();
		if (picked == null) {
			DriverStation.reportError("WARNING: No auto command detected, defaulting to baseline auto.", false);
			return new DriveTimeCommand(2.2, 0.3, driveSubsystem);
		} else {
			return picked;
		}
	}

	public DriveSubsystem getDriveSubsystem() {
		return this.driveSubsystem;
	}
}
