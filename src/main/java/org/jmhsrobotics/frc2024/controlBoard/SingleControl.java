package org.jmhsrobotics.frc2024.controlBoard;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SingleControl implements ControlBoard {
	private XboxController driver = new XboxController(0);

	// =============Driver Controls=============
	@Override
	public double xInput() {
		return this.driver.getLeftX();
	}

	@Override
	public double yInput() {
		return this.driver.getLeftY();
	}

	@Override
	public double rotationalInput() {
		return this.driver.getRightX();
	}

	@Override
	public Trigger brake() {
		return new JoystickButton(this.driver, XboxController.Button.kLeftBumper.value);
	}

	@Override
	public Trigger setZeroHeading() {
		return new JoystickButton(this.driver, XboxController.Button.kBack.value);
	}

	public double pitchInput() {
		return 0;
	}

	public Trigger presetHigh() {
		return new JoystickButton(this.driver, XboxController.Button.kY.value);
	}

	@Override
	public Trigger presetMid() {
		return new JoystickButton(this.driver, XboxController.Button.kB.value);
	}

	public Trigger presetLow() {
		return new JoystickButton(this.driver, XboxController.Button.kA.value);
	}

	@Override
	public Trigger intakeInput() {
		return new Trigger(() -> {
			return this.driver.getRightTriggerAxis() > 0.5;
		}); // Right Trigger
	}

	@Override
	public Trigger extakeInput() {
		return new Trigger(() -> {
			return driver.getLeftTriggerAxis() > 0.5;
		}); // Left Trigger
	}

	@Override
	public Trigger shooterInput() {
		return new JoystickButton(this.driver, XboxController.Button.kRightBumper.value);
	}

	@Override
	public Trigger ampShooterInput() {
		return new JoystickButton(this.driver, XboxController.Button.kX.value);
	}

	@Override
	public Trigger climberExtend() {
		return new Trigger(() -> {
			return this.driver.getPOV() == 0;
		}); // DPAD Up
	}

	@Override
	public Trigger climberRetract() {
		return new Trigger(() -> {
			return this.driver.getPOV() == 180;
		}); // DPAD Down
	}

	@Override
	public Trigger AprilLockOn() {
		return new Trigger(driver::getStartButton);
	}

	@Override
	public void setRumble(RumbleType type, double value) {
		this.driver.setRumble(type, value);
	}

	@Override
	public Trigger ObjectLockOn() {
		// TODO Auto-generated method stub
		throw new UnsupportedOperationException("Unimplemented method 'ObjectLockOn'");
	}
}
