// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.jmhsrobotics.frc2024.subsystems.drive.swerve;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import org.jmhsrobotics.frc2024.Constants;
import org.jmhsrobotics.frc2024.utils.SparkMaxConfigUtils;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class MAXSwerveModule implements ISwerveModule {
	private SparkMax m_drivingSparkMax;
	private SparkMax m_turningSparkMax;
	private SparkMaxConfig m_drivingSparkMaxConfig;
	private SparkMaxConfig m_turningSparkMaxConfig;

	private RelativeEncoder m_drivingEncoder;
	private AbsoluteEncoder m_turningEncoder;
	private AbsoluteEncoderConfig m_drivingEncoderConfig;
	private AbsoluteEncoderConfig m_turningEncoderConfig;

	private SparkClosedLoopController m_drivingPIDController;
	private SparkClosedLoopController m_turningPIDController;
	private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.08384, 2.4421, 0.33689);
	private double m_chassisAngularOffset = 0;
	private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

	/**
	 * Constructs a MAXSwerveModule and configures the driving and turning motor,
	 * encoder, and PID controller. This configuration is specific to the REV
	 * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore Encoder.
	 */
	public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
		m_drivingSparkMax = new SparkMax(drivingCANId, MotorType.kBrushless);
		m_turningSparkMax = new SparkMax(turningCANId, MotorType.kBrushless);
		// Factory reset, so we get the SPARKS MAX to a known state before configuring
		// them. This is useful in case a SPARK MAX is swapped out.

		// Setup encoders and PID controllers for the driving and turning SPARKS MAX.
		m_drivingEncoder = m_drivingSparkMax.getEncoder();
		m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder();
		m_drivingPIDController = m_drivingSparkMax.getClosedLoopController();
		// m_drivingPIDController = m_drivingSparkMax.getPIDController();
		// m_turningPIDController = m_turningSparkMax.getPIDController();
		// m_drivingPIDController.setFeedbackDevice(m_drivingEncoder);
		// m_turningPIDController.setFeedbackDevice(m_turningEncoder);

		m_drivingSparkMaxConfig.idleMode(Constants.ModuleConstants.kDrivingMotorIdleMode)
				.smartCurrentLimit(Constants.ModuleConstants.kDrivingMotorCurrentLimit);

		m_drivingSparkMaxConfig.encoder.uvwAverageDepth(2).uvwMeasurementPeriod(16)
				.positionConversionFactor(Constants.ModuleConstants.kDrivingEncoderPositionFactor)
				.velocityConversionFactor(Constants.ModuleConstants.kDrivingEncoderVelocityFactor);

		m_drivingSparkMaxConfig.closedLoop
				.pidf(Constants.ModuleConstants.kDrivingP, Constants.ModuleConstants.kDrivingI,
						Constants.ModuleConstants.kDrivingD, Constants.ModuleConstants.kDrivingFF)
				.outputRange(Constants.ModuleConstants.kDrivingMinOutput, Constants.ModuleConstants.kDrivingMaxOutput);

		m_turningSparkMaxConfig.idleMode(Constants.ModuleConstants.kTurningMotorIdleMode)
				.smartCurrentLimit(Constants.ModuleConstants.kTurningMotorCurrentLimit);

		m_turningSparkMaxConfig.absoluteEncoder
				.positionConversionFactor(Constants.ModuleConstants.kTurningEncoderPositionFactor)
				.velocityConversionFactor(Constants.ModuleConstants.kTurningEncoderVelocityFactor)
				.inverted(Constants.ModuleConstants.kTurningEncoderInverted);

		m_turningSparkMaxConfig.closedLoop
				.pidf(Constants.ModuleConstants.kTurningP, Constants.ModuleConstants.kTurningI,
						Constants.ModuleConstants.kTurningD, Constants.ModuleConstants.kTurningFF)
				.outputRange(Constants.ModuleConstants.kTurningMinOutput, Constants.ModuleConstants.kTurningMaxOutput);

		SparkMaxConfigUtils.applyConfigInFlight(m_drivingSparkMax, m_drivingSparkMaxConfig);
		SparkMaxConfigUtils.applyConfigInFlight(m_turningSparkMax, m_turningSparkMaxConfig);

		// Apply position and velocity conversion factors for the driving encoder. The
		// native units for position and velocity are rotations and RPM, respectively,
		// but we want meters and meters per second to use with WPILib's swerve APIs.
		// m_drivingEncoder.setPositionConversionFactor(Constants.ModuleConstants.kDrivingEncoderPositionFactor);
		// m_drivingEncoder.setVelocityConversionFactor(Constants.ModuleConstants.kDrivingEncoderVelocityFactor);

		// Apply position and velocity conversion factors for the turning encoder. We
		// want these in radians and radians per second to use with WPILib's swerve
		// APIs.
		// m_turningEncoder.setPositionConversionFactor(Constants.ModuleConstants.kTurningEncoderPositionFactor);
		// m_turningEncoder.setVelocityConversionFactor(Constants.ModuleConstants.kTurningEncoderVelocityFactor);

		// Invert the turning encoder, since the output shaft rotates in the opposite
		// direction of
		// the steering motor in the MAXSwerve Module.
		// m_turningEncoder.setInverted(Constants.ModuleConstants.kTurningEncoderInverted);

		// Enable PID wrap around for the turning motor. This will allow the PID
		// controller to go through 0 to get to the setpoint i.e. going from 350 degrees
		// to 10 degrees will go through 0 rather than the other direction which is a
		// longer route.

		// m_turningPIDController.setPositionPIDWrappingEnabled(true);
		// m_turningPIDController
		// .setPositionPIDWrappingMinInput(Constants.ModuleConstants.kTurningEncoderPositionPIDMinInput);
		// m_turningPIDController
		// .setPositionPIDWrappingMaxInput(Constants.ModuleConstants.kTurningEncoderPositionPIDMaxInput);
		// Set the PID gains for the driving motor. Note these are example gains, and
		// you
		// may need to tune them for your own robot!
		// m_drivingPIDController.setP(Constants.ModuleConstants.kDrivingP);
		// m_drivingPIDController.setI(Constants.ModuleConstants.kDrivingI);
		// m_drivingPIDController.setD(Constants.ModuleConstants.kDrivingD);
		// m_drivingPIDController.setFF(Constants.ModuleConstants.kDrivingFF);
		// m_drivingPIDController.setOutputRange(Constants.ModuleConstants.kDrivingMinOutput,
		// Constants.ModuleConstants.kDrivingMaxOutput);

		// Set the PID gains for the turning motor. Note these are example gains, and
		// you
		// may need to tune them for your own robot!
		// m_turningPIDController.setP(Constants.ModuleConstants.kTurningP);
		// m_turningPIDController.setI(Constants.ModuleConstants.kTurningI);
		// m_turningPIDController.setD(Constants.ModuleConstants.kTurningD);
		// m_turningPIDController.setFF(Constants.ModuleConstants.kTurningFF);
		// m_turningPIDController.setOutputRange(Constants.ModuleConstants.kTurningMinOutput,
		// Constants.ModuleConstants.kTurningMaxOutput);

		// m_drivingSparkMax.setIdleMode(Constants.ModuleConstants.kDrivingMotorIdleMode);
		// m_turningSparkMax.setIdleMode(Constants.ModuleConstants.kTurningMotorIdleMode);
		// m_drivingSparkMax.setSmartCurrentLimit(Constants.ModuleConstants.kDrivingMotorCurrentLimit);
		// m_turningSparkMax.setSmartCurrentLimit(Constants.ModuleConstants.kTurningMotorCurrentLimit);

		// Save the SPARK MAX configurations. If a SPARK MAX browns out during
		// operation, it will maintain the above configurations.
		// m_drivingSparkMax.burnFlash();
		// m_turningSparkMax.burnFlash();

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
		m_drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, ControlType.kVelocity,
				ClosedLoopSlot.kSlot0, ff, ArbFFUnits.kVoltage);

		// optimizedDesiredState = correctedDesiredState;
		// Command driving and turning SPARKS MAX towards their respective setpoints.
		m_drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, SparkMax.ControlType.kVelocity);
		m_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), SparkMax.ControlType.kPosition);

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
