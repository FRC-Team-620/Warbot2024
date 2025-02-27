package org.jmhsrobotics.frc2024.ComboCommands;

import org.jmhsrobotics.frc2024.Constants;
import org.jmhsrobotics.frc2024.subsystems.arm.ArmPIDSubsystem;
import org.jmhsrobotics.frc2024.subsystems.arm.commands.CommandArm;
import org.jmhsrobotics.frc2024.subsystems.intake.IntakeSubsystem;
import org.jmhsrobotics.frc2024.subsystems.intake.commands.IntakeCommand;
import org.jmhsrobotics.frc2024.subsystems.shooter.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class ComboIntakeArmCommand extends ParallelCommandGroup {

	/**
	 * Moves arm to the Pickup position and Intakes.
	 *
	 * @param arm
	 * @param shooter
	 * @param intake
	 */
	public ComboIntakeArmCommand(ArmPIDSubsystem arm, ShooterSubsystem shooter, IntakeSubsystem intake) {
		addCommands(new CommandArm(arm, Constants.ArmSetpoint.PICKUP.value), // move arm to intake position
				new IntakeCommand(1, intake, shooter)// start intkae
		);
	}

}
