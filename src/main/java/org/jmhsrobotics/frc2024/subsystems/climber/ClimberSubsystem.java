package org.jmhsrobotics.frc2024.subsystems.climber;

import org.jmhsrobotics.frc2024.Constants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
	private SparkMax leftClimber, rightClimber;
	private SparkMaxConfig leftClimberConfig = new SparkMaxConfig();
	private SparkMaxConfig rightClimberConfig = new SparkMaxConfig();

	public ClimberSubsystem() {
		leftClimber = new SparkMax(Constants.CAN.kLeftClimberID, MotorType.kBrushless);
		rightClimber = new SparkMax(Constants.CAN.kRightClimberID, MotorType.kBrushless);

		// Set the default configuration for the left climber
		leftClimberConfig.smartCurrentLimit(40);
		leftClimberConfig.softLimit.reverseSoftLimit(0);
		leftClimberConfig.softLimit.forwardSoftLimit(34);
		leftClimberConfig.softLimit.forwardSoftLimitEnabled(true);
		leftClimberConfig.idleMode(IdleMode.kBrake);
		leftClimberConfig.encoder.positionConversionFactor((5.0 * 4.0) / 100.0); // 20:1 gear reduction

		// Set the default configuration for the right climber
		rightClimberConfig.smartCurrentLimit(40);
		rightClimberConfig.softLimit.reverseSoftLimit(0);
		rightClimberConfig.softLimit.forwardSoftLimit(34);
		rightClimberConfig.softLimit.forwardSoftLimitEnabled(true);
		rightClimberConfig.idleMode(IdleMode.kBrake);
		rightClimberConfig.encoder.positionConversionFactor((5.0 * 4.0) / 100.0); // 20:1 gear reduction

		// Apply the configurations
		leftClimber.configure(leftClimberConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
		rightClimber.configure(leftClimberConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

		// TODO: input real vals for soft limit
		SmartDashboard.putNumber("climber/leftEncoder", 0);
		SmartDashboard.putNumber("climber/rightEncoder", 0);

		// climber.setInverted(true);

		// rightClimber.follow(leftClimber, true);
	}

	public void setSoftLimit(boolean toggle) {
		// Update the configuration
		this.leftClimberConfig.softLimit.forwardSoftLimitEnabled(toggle);
		this.leftClimberConfig.softLimit.reverseSoftLimitEnabled(toggle);
		this.rightClimberConfig.softLimit.forwardSoftLimitEnabled(toggle);
		this.rightClimberConfig.softLimit.reverseSoftLimitEnabled(toggle);

		// Apply the changes
		this.leftClimber.configure(this.leftClimberConfig, ResetMode.kNoResetSafeParameters,
				PersistMode.kNoPersistParameters);
		this.rightClimber.configure(this.leftClimberConfig, ResetMode.kNoResetSafeParameters,
				PersistMode.kNoPersistParameters);
	}

	public void setLeftMotor(double amount) {
		this.leftClimber.set(amount);
	}

	public void setRightMotor(double amount) {
		this.rightClimber.set(amount);
	}

	public void climberRetract() {
		this.leftClimber.setVoltage(-10);
		this.rightClimber.setVoltage(-10);
		SmartDashboard.putNumber("climber/leftEncoder", getLeftEncoderPostition());
		SmartDashboard.putNumber("climber/rightEncoder", getRightEncoderPosition());
	}

	public void climberExtend() {
		this.leftClimber.setVoltage(10);
		this.rightClimber.setVoltage(10);
		SmartDashboard.putNumber("climber/leftEncoder", getLeftEncoderPostition());
		SmartDashboard.putNumber("climber/rightEncoder", getRightEncoderPosition());
	}

	public void climberStop() {
		this.leftClimber.setVoltage(0);
		this.rightClimber.setVoltage(0);
	}

	// Returns the current position of the left climber
	public double getLeftEncoderPostition() {
		return this.leftClimber.getEncoder().getPosition();
	}

	// Returns the current position of the right climber
	public double getRightEncoderPosition() {
		return this.rightClimber.getEncoder().getPosition();
	}

	@Override
	public void periodic() {
		super.periodic();
	}
}
