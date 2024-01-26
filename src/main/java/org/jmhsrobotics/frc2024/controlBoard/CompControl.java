package org.jmhsrobotics.frc2024.controlBoard;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CompControl implements ControlBoard {
	private CommandXboxController driver = new CommandXboxController(0);
	private CommandXboxController operator = new CommandXboxController(1);

	// TODO: Implement operator controls in the future
	// =============Operator Controls=============

	// =============Driver Controls=============
	@Override
	public double xInput() {
		// TODO Auto-generated method stub
		return this.driver.getLeftX();
	}

	@Override
	public double yInput() {
		// TODO Auto-generated method stub
		return this.driver.getLeftY();
	}

	@Override
	public double rotationalInput() {
		// TODO Auto-generated method stub
		return this.driver.getRightX();
	}

	public double pitchInput() {
		// TODO Auto-generated method stub
		return this.driver.getRightY();
	}

	@Override
	public Trigger brake() {
		// TODO Auto-generated method stub
		return this.driver.leftBumper();
	}

	@Override
	public Trigger setZeroHeading() {
		// TODO Auto-generated method stub
		return this.driver.rightBumper();
	}

	public Trigger presetHigh() {
		// TODO Auto-generated method stub
		return this.driver.y();
	}

	public Trigger presetLow() {
		// TODO Auto-generated method stub
		return this.driver.a();
	}

}
