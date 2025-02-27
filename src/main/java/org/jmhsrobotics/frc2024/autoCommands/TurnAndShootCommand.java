package org.jmhsrobotics.frc2024.autoCommands;

import org.jmhsrobotics.frc2024.Constants;
import org.jmhsrobotics.frc2024.subsystems.arm.ArmPIDSubsystem;
import org.jmhsrobotics.frc2024.subsystems.arm.commands.CommandArm;
import org.jmhsrobotics.frc2024.subsystems.drive.DriveSubsystem;
import org.jmhsrobotics.frc2024.subsystems.drive.commands.LockSpeaker;
import org.jmhsrobotics.frc2024.subsystems.intake.IntakeSubsystem;
import org.jmhsrobotics.frc2024.subsystems.shooter.ShooterSubsystem;
import org.jmhsrobotics.frc2024.subsystems.vision.VisionSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TurnAndShootCommand extends SequentialCommandGroup {

	private VisionSubsystem visionSubsystem;
	private DriveSubsystem driveSubsystem;
	private ArmPIDSubsystem armSubsystem;
	private ShooterSubsystem shooterSubsystem;
	private IntakeSubsystem intakeSubsystem;

	public TurnAndShootCommand(VisionSubsystem visionSubsystem, DriveSubsystem driveSubsystem,
			ArmPIDSubsystem armSubsystem, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
		// TODO: rename the folder name
		this.visionSubsystem = visionSubsystem;
		this.driveSubsystem = driveSubsystem;
		this.armSubsystem = armSubsystem;
		this.shooterSubsystem = shooterSubsystem;
		this.intakeSubsystem = intakeSubsystem;

		addCommands(new ParallelCommandGroup(new LockSpeaker(this.driveSubsystem, this.visionSubsystem), // TODO:
																											// Remove
																											// hardcoded
																											// id
				new CommandArm(this.armSubsystem, Constants.ArmSetpoint.SHOOT.value)),
				new FireCommand(this.intakeSubsystem, this.shooterSubsystem));

		addRequirements(this.visionSubsystem, this.armSubsystem, this.driveSubsystem, this.shooterSubsystem,
				this.intakeSubsystem);
	}

}
