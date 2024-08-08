package org.jmhsrobotics.frc2024.controlBoard;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SwitchableControlBoard implements ControlBoard {

	ControlBoard current;

	// Hacky Class to allow for real time switching of control boards
	public SwitchableControlBoard(ControlBoard board) {
		current = board;
	}
	public void setControlBoard(ControlBoard board) {
		current = board;
	}
	@Override
	public Trigger presetHigh() {
		return current.presetHigh();
	}

	@Override
	public Trigger presetMid() {
		return current.presetMid();
	}

	@Override
	public Trigger presetLow() {
		return current.presetLow();
	}

	@Override
	public Trigger brake() {
		return current.brake();
	}

	@Override
	public Trigger setZeroHeading() {
		return current.setZeroHeading();
	}

	@Override
	public double xInput() {
		return current.xInput();
	}

	@Override
	public double yInput() {
		return current.yInput();
	}

	@Override
	public double rotationalInput() {
		return current.rotationalInput();
	}

	@Override
	public double pitchInput() {
		return current.pitchInput();
	}

	@Override
	public Trigger intakeInput() {
		return current.intakeInput();
	}

	@Override
	public Trigger shooterInput() {
		return current.shooterInput();
	}

	@Override
	public Trigger extakeInput() {
		return current.extakeInput();
	}

	@Override
	public Trigger ampShooterInput() {
		return current.ampShooterInput();
	}

	@Override
	public Trigger climberExtend() {
		return current.climberExtend();
	}

	@Override
	public Trigger climberRetract() {
		return current.climberRetract();
	}

	@Override
	public Trigger AprilLockOn() {
		return current.AprilLockOn();
	}
	@Override
	public void setRumble(RumbleType type, double value) {
		// TODO Auto-generated method stub
		current.setRumble(type, value);
	}
	@Override
	public Trigger ObjectLockOn() {
		// TODO Auto-generated method stub
		throw new UnsupportedOperationException("Unimplemented method 'ObjectLockOn'");
	}

}
