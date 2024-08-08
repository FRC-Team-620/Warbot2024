package org.jmhsrobotics.frc2024.controlBoard;

import org.jmhsrobotics.frc2024.Constants;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CompControl implements ControlBoard {
	private XboxController driver = new XboxController(0);
	private XboxController operator = new XboxController(1);

	// =============Driver Controls=============
	private double slowSpeedFps = Constants.SwerveConstants.slowSpeedFeetPerSecond;
	private double slowMode = slowSpeedFps / Constants.SwerveConstants.kMaxSpeedFeetPerSecond;

	private boolean isTurbo() {
		// return this.driver.getAButton();
		// return new Trigger(() -> {
		return driver.getRightTriggerAxis() > 0.5;
		// }); //
	}

	@Override
	public double xInput() {
		return this.driver.getLeftX() * (isTurbo() ? 1.0 : slowMode);
	}

	@Override
	public double yInput() {
		return this.driver.getLeftY() * (isTurbo() ? 1.0 : slowMode);
	}

	@Override
	public double rotationalInput() {
		return this.driver.getRightX() * 0.7;
	}

	@Override
	public Trigger brake() {
		return new JoystickButton(this.driver, XboxController.Button.kLeftBumper.value);
	}

	@Override
	public Trigger setZeroHeading() {
		return new JoystickButton(this.driver, XboxController.Button.kRightBumper.value);
	}

	@Override
	public Trigger AprilLockOn() {
		return new Trigger(() -> {
			return this.driver.getLeftTriggerAxis() > 0.5;
		});
	}

	@Override
	public Trigger ObjectLockOn() {
		return new Trigger(() -> {
			return this.driver.getRightTriggerAxis() > 0.5;
		});
	}
	// =============Operator Controls=============
	public double pitchInput() {
		return this.operator.getRightY();
	}

	public Trigger presetHigh() {
		return new JoystickButton(this.operator, XboxController.Button.kY.value);

	}

	@Override
	public Trigger presetMid() {
		return new JoystickButton(this.operator, XboxController.Button.kB.value);
	}

	public Trigger presetLow() {
		return new JoystickButton(this.operator, XboxController.Button.kA.value);
	}

	@Override
	public Trigger intakeInput() {
		return new Trigger(() -> {
			return this.operator.getRightTriggerAxis() > 0.5;
		}); // Right Trigger
	}

	@Override
	public Trigger extakeInput() {
		return new Trigger(() -> {
			return operator.getLeftTriggerAxis() > 0.5;
		}); // Left Trigger
	}

	@Override
	public Trigger shooterInput() {
		return new JoystickButton(this.operator, XboxController.Button.kRightBumper.value);
	}

	@Override
	public Trigger ampShooterInput() {
		return new JoystickButton(this.operator, XboxController.Button.kX.value);
	}

	@Override
	public Trigger climberExtend() {
		return new Trigger(() -> {
			return this.operator.getPOV() == 0;
		}); // DPAD Upe
	}

	@Override
	public Trigger climberRetract() {
		return new Trigger(() -> {
			return this.operator.getPOV() == 180;
		}); // DPAD Down
	}

	@Override
	public void setRumble(RumbleType type, double value) {
		// TODO Auto-generated method stub
		this.driver.setRumble(type, value);
		this.operator.setRumble(type, value);
	}
}
