package org.jmhsrobotics.frc2024;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {

	/**
	 * CAN id assignments
	 */
	public static class CAN {
		public static final int kIntakeId = 40;
		public static final int kShooterTopId = 30;
		public static final int kShooterBottomId = 31;
		public static final int kArmPivotRightID = 50;
		public static final int kArmPivotFollowerID = 51;
		public static final int kLeftClimberID = 60;
		public static final int kRightClimberID = 61;
		public static final int kMaxFramePeriodMs = 32767;

	};

	/**
	 * Digital IO ports
	 */
	public static class DIO {
		public static final int kIntakeSwitch = 2;
	}

	/**
	 * PWM output ports
	 */
	public static class PWM {
		public static final int kLedStrip = 9;
	}

	public static class Arm {
		public static double pickupSetpoint = 3;
		public static double shootingSetpoint = 11.5;
		public static double ampSetpoint = 93;
	}

	public static enum ArmSetpoint {
		PICKUP(Arm.pickupSetpoint), SHOOT(Arm.shootingSetpoint), AMP(Arm.ampSetpoint);

		public final double value;

		private ArmSetpoint(double value) {
			this.value = value;
		}
	}

	public static class SwerveConstants {
		// SPARK MAX CAN IDs
		public static final int kFrontLeftDrivingCanId = 10;
		public static final int kRearLeftDrivingCanId = 12;
		public static final int kFrontRightDrivingCanId = 11;
		public static final int kRearRightDrivingCanId = 13;

		public static final int kFrontLeftTurningCanId = 20;
		public static final int kRearLeftTurningCanId = 22;
		public static final int kFrontRightTurningCanId = 21;
		public static final int kRearRightTurningCanId = 23;

		public static final int kGyroCanId = 62;
		public static final boolean kGyroReversed = false;

		public static final double dtOffset = 0.02;

		public static final double kMaxXSpeed = 1;
		public static final double kMaxYSpeed = 1;
		public static final double kMaxRotationSpeed = 1;
		public static final boolean kFieldRelative = true;
		public static final boolean kRateLimit = false;

		// drive deadband constants
		public static final double kDeadBand = 0.05;
		// Driving Parameters - Note that these are not the maximum capable speeds of
		// the robot, rather the allowed maximum speeds
		// public static final double kMaxSpeedMetersPerSecond = 4.2;

		/* START DRIVER CONFIGURABLES */

		// configure to change drivetrain turbo mode
		public static final double kMaxSpeedFeetPerSecond = 12;

		// configure to change drivetrain slow mode
		public static final double slowSpeedFeetPerSecond = 6; // TODO: Remove used when not in turbo mode move to
																// Comp
																// control later after refactor

		// configure to change rotation speed
		public static final double kMaxAngularSpeed = 6; // radians per second

		/* END DRIVER CONFIGURABLES */

		// DO NOT change
		public static final double kMaxSpeedMetersPerSecond = 0.3048 * kMaxSpeedFeetPerSecond;

		public static final double kDirectionSlewRate = 1.4; // radians per second
		public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
		public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

		private static final double kRobotWidthWheelDifference = 1.75 * 2;

		// Chassis configuration
		// Distance between centers of right and left wheels on robot
		public static final double kTrackWidth = Units.inchesToMeters(26 - kRobotWidthWheelDifference);
		// Distance between centers of front and back wheels on robot
		public static final double kWheelBase = Units.inchesToMeters(26 - kRobotWidthWheelDifference);

		public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
				new Translation2d(kWheelBase / 2, kTrackWidth / 2), new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
				new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
				new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

		// Angular offsets of the modules relative to the chassis in radians
		public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
		public static final double kFrontRightChassisAngularOffset = 0;
		public static final double kBackLeftChassisAngularOffset = Math.PI;
		public static final double kBackRightChassisAngularOffset = Math.PI / 2;
	}

	public enum ModuleSpeed {
		HIGH(14), MEDIUM(13), LOW(12);

		public final int pinionTeeth;

		ModuleSpeed(int pinionTeeth) {
			this.pinionTeeth = pinionTeeth;
		}

	}

	public static final class ModuleConstants {
		// The MAXSwerve module can be configured with one of three pinion gears: 12T,
		// 13T, or 14T.
		// This changes the drive speed of the module (a pinion gear with more teeth
		// will result in a
		// robot that drives faster).
		public static final int kDrivingMotorPinionTeeth = ModuleSpeed.HIGH.pinionTeeth;

		// Invert the turning encoder, since the output shaft rotates in the opposite
		// direction of
		// the steering motor in the MAXSwerve Module.
		public static final boolean kTurningEncoderInverted = true;

		// Calculations required for driving motor conversion factors and feed forward
		public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
		public static final double kWheelDiameterMeters = 0.0762;
		public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
		// 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
		// teeth on the bevel pinion
		public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
		public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
				/ kDrivingMotorReduction;

		// used
		public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
				/ kDrivingMotorReduction; // meters
		// used
		public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
				/ kDrivingMotorReduction) / 60.0; // meters per second

		public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
		public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

		public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
		public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

		public static final double kDrivingP = 0.2;
		public static final double kDrivingI = 0;
		public static final double kDrivingD = 0.1;
		public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
		public static final double kDrivingMinOutput = -1;
		public static final double kDrivingMaxOutput = 1;

		public static final double kTurningP = 2;
		public static final double kTurningI = 0;
		public static final double kTurningD = 0.5;
		public static final double kTurningFF = 0;
		public static final double kTurningMinOutput = -1;
		public static final double kTurningMaxOutput = 1;

		public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
		public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

		public static final int kDrivingMotorCurrentLimit = 50; // amps
		public static final int kTurningMotorCurrentLimit = 20; // amps
	}

	// TODO: Clean up auto constants later
	public static final class AutoConstants {

	}

	public static final class NeoMotorConstants {
		public static final double kFreeSpeedRpm = 5676;
	}

	public static final class LEDConstants {
		public static final int LEDPortID = 0;
		public static final int LEDLength = 65;
		public static final int rainbowSpeed = 3;
	}

	public static final double ksimDtSec = 0.02;
}
