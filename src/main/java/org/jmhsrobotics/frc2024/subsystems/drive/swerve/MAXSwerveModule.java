// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.jmhsrobotics.frc2024.subsystems.drive.swerve;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import org.jmhsrobotics.frc2024.Constants;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

public class MAXSwerveModule implements ISwerveModule {
	private final CANSparkMax m_drivingSparkMax;
	private final CANSparkMax m_turningSparkMax;

	private final RelativeEncoder m_drivingEncoder;
	private final AbsoluteEncoder m_turningEncoder;

	// private final SparkMaxPIDController m_drivingPIDController;
	// private final SparkMaxPIDController m_turningPIDController;
	private final SparkPIDController m_drivingPIDController;
	private final SparkPIDController m_turningPIDController;
	private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.08384, 2.4421, 0.33689);
	private double m_chassisAngularOffset = 0;
	private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

	/**
	 * Constructs a MAXSwerveModule and configures the driving and turning motor,
	 * encoder, and PID controller. This configuration is specific to the REV
	 * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore Encoder.
	 */
	public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
		m_drivingSparkMax = new CANSparkMax(drivingCANId, MotorType.kBrushless);
		m_turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);
		// Factory reset, so we get the SPARKS MAX to a known state before configuring
		// them. This is useful in case a SPARK MAX is swapped out.
		m_drivingSparkMax.restoreFactoryDefaults();
		m_turningSparkMax.restoreFactoryDefaults();

		// Setup encoders and PID controllers for the driving and turning SPARKS MAX.
		m_drivingEncoder = m_drivingSparkMax.getEncoder();
		m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
		m_drivingPIDController = m_drivingSparkMax.getPIDController();
		m_turningPIDController = m_turningSparkMax.getPIDController();
		m_drivingPIDController.setFeedbackDevice(m_drivingEncoder);
		m_turningPIDController.setFeedbackDevice(m_turningEncoder);

		m_drivingEncoder.setAverageDepth(2);
		m_drivingEncoder.setMeasurementPeriod(16);
		// Apply position and velocity conversion factors for the driving encoder. The
		// native units for position and velocity are rotations and RPM, respectively,
		// but we want meters and meters per second to use with WPILib's swerve APIs.
		m_drivingEncoder.setPositionConversionFactor(Constants.ModuleConstants.kDrivingEncoderPositionFactor);
		m_drivingEncoder.setVelocityConversionFactor(Constants.ModuleConstants.kDrivingEncoderVelocityFactor);

		// Apply position and velocity conversion factors for the turning encoder. We
		// want these in radians and radians per second to use with WPILib's swerve
		// APIs.
		m_turningEncoder.setPositionConversionFactor(Constants.ModuleConstants.kTurningEncoderPositionFactor);
		m_turningEncoder.setVelocityConversionFactor(Constants.ModuleConstants.kTurningEncoderVelocityFactor);

		// Invert the turning encoder, since the output shaft rotates in the opposite
		// direction of
		// the steering motor in the MAXSwerve Module.
		m_turningEncoder.setInverted(Constants.ModuleConstants.kTurningEncoderInverted);

		// Enable PID wrap around for the turning motor. This will allow the PID
		// controller to go through 0 to get to the setpoint i.e. going from 350 degrees
		// to 10 degrees will go through 0 rather than the other direction which is a
		// longer route.
		m_turningPIDController.setPositionPIDWrappingEnabled(true);
		m_turningPIDController
				.setPositionPIDWrappingMinInput(Constants.ModuleConstants.kTurningEncoderPositionPIDMinInput);
		m_turningPIDController
				.setPositionPIDWrappingMaxInput(Constants.ModuleConstants.kTurningEncoderPositionPIDMaxInput);

		// Set the PID gains for the driving motor. Note these are example gains, and
		// you
		// may need to tune them for your own robot!
		m_drivingPIDController.setP(Constants.ModuleConstants.kDrivingP);
		m_drivingPIDController.setI(Constants.ModuleConstants.kDrivingI);
		m_drivingPIDController.setD(Constants.ModuleConstants.kDrivingD);
		m_drivingPIDController.setFF(Constants.ModuleConstants.kDrivingFF);
		m_drivingPIDController.setOutputRange(Constants.ModuleConstants.kDrivingMinOutput,
				Constants.ModuleConstants.kDrivingMaxOutput);

		// Set the PID gains for the turning motor. Note these are example gains, and
		// you
		// may need to tune them for your own robot!
		m_turningPIDController.setP(Constants.ModuleConstants.kTurningP);
		m_turningPIDController.setI(Constants.ModuleConstants.kTurningI);
		m_turningPIDController.setD(Constants.ModuleConstants.kTurningD);
		m_turningPIDController.setFF(Constants.ModuleConstants.kTurningFF);
		m_turningPIDController.setOutputRange(Constants.ModuleConstants.kTurningMinOutput,
				Constants.ModuleConstants.kTurningMaxOutput);

		m_drivingSparkMax.setIdleMode(Constants.ModuleConstants.kDrivingMotorIdleMode);
		m_turningSparkMax.setIdleMode(Constants.ModuleConstants.kTurningMotorIdleMode);
		m_drivingSparkMax.setSmartCurrentLimit(Constants.ModuleConstants.kDrivingMotorCurrentLimit);
		m_turningSparkMax.setSmartCurrentLimit(Constants.ModuleConstants.kTurningMotorCurrentLimit);

		// Save the SPARK MAX configurations. If a SPARK MAX browns out during
		// operation, it will maintain the above configurations.
		m_drivingSparkMax.burnFlash();
		m_turningSparkMax.burnFlash();

		m_chassisAngularOffset = chassisAngularOffset;
		m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
		m_drivingEncoder.setPosition(0);
	}

	/**
	 * Returns the current state of the module.
	 *
	 * @return The current state of the module.
	 */
	public SwerveModuleState getState() {
		// Apply chassis angular offset to the encoder position to get the position
		// relative to the chassis.
		return new SwerveModuleState(m_drivingEncoder.getVelocity(),
				new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
	}

	public SwerveModuleState getDesiredState() {
		return m_desiredState;
	}

	/**
	 * Returns the current position of the module.
	 *
	 * @return The current position of the module.
	 */
	public SwerveModulePosition getPosition() {
		// Apply chassis angular offset to the encoder position to get the position
		// relative to the chassis.
		return new SwerveModulePosition(m_drivingEncoder.getPosition(),
				new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
	}

	/**
	 * Sets the desired state for the module.
	 *
	 * @param desiredState
	 *            Desired state with speed and angle.
	 */
	public void setDesiredState(SwerveModuleState desiredState) {
		// Apply chassis angular offset to the desired state.
		SwerveModuleState correctedDesiredState = new SwerveModuleState();
		correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
		correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

		// Optimize the reference state to avoid spinning further than 90 degrees.
		SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
				new Rotation2d(m_turningEncoder.getPosition()));

		double ff = feedforward.calculate(optimizedDesiredState.speedMetersPerSecond);
		m_drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, ControlType.kVelocity, 0, ff,
				ArbFFUnits.kVoltage);

		// optimizedDesiredState = correctedDesiredState;
		// Command driving and turning SPARKS MAX towards their respective setpoints.
		m_drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond,
				CANSparkMax.ControlType.kVelocity);
		m_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(),
				CANSparkMax.ControlType.kPosition);

		m_desiredState = optimizedDesiredState;
	}

	/** Zeroes all the SwerveModule encoders. */
	public void resetEncoders() {
		m_drivingEncoder.setPosition(0);
	}

	@Override
	public void update(double dt) {
		// TODO Auto-generated method stub

	}

}
