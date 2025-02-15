package org.jmhsrobotics.frc2024.subsystems.drive.swerve;

import org.jmhsrobotics.frc2024.Constants;
import org.jmhsrobotics.frc2024.Constants.SwerveConstants;
import org.jmhsrobotics.frc2024.utils.SwerveUtils;
import org.jmhsrobotics.warcore.swerve.SwerveVisualizer;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class RevSwerveDrive {

	// Create MAXSwerveModules
	private ISwerveModule m_frontLeft, m_frontRight, m_rearLeft, m_rearRight;

	private final Pigeon2 m_gyro;

	// Slew rate filter variables for controlling lateral acceleration
	private double m_currentRotation = 0.0;
	private double m_currentTranslationDir = 0.0;
	private double m_currentTranslationMag = 0.0;

	private SwerveVisualizer m_visualizer;

	private SlewRateLimiter m_magLimiter = new SlewRateLimiter(Constants.SwerveConstants.kMagnitudeSlewRate);
	private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(Constants.SwerveConstants.kRotationalSlewRate);
	private double m_prevTime = WPIUtilJNI.now() * 1e-6;

	// Odometry class for tracking robot pose
	private SwerveDriveOdometry m_odometry;

	public RevSwerveDrive(ISwerveModule frontLeft, ISwerveModule frontRight, ISwerveModule rearLeft,
			ISwerveModule rearRight, Pigeon2 gyro) {

		this.m_frontLeft = frontLeft;
		this.m_frontRight = frontRight;
		this.m_rearLeft = rearLeft;
		this.m_rearRight = rearRight;

		this.m_gyro = gyro;

		this.m_odometry = new SwerveDriveOdometry(Constants.SwerveConstants.kDriveKinematics, getRawGyro(),
				new SwerveModulePosition[]{m_frontLeft.getPosition(), m_frontRight.getPosition(),
						m_rearLeft.getPosition(), m_rearRight.getPosition()});

		m_visualizer = new SwerveVisualizer(Constants.SwerveConstants.kTrackWidth,
				Constants.SwerveConstants.kWheelBase);
	}

	/**
	 * Returns the current yaw of the robot.
	 *
	 * @return the current yaw of the robot
	 */
	private Rotation2d getRawGyro() {
		// return this.getPose().getRotation();
		// return Rotation2d.fromDegrees(m_gyro.getYaw().getValue());
		return Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble());
	}

	/**
	 * Returns the heading of the robot.
	 *
	 * @return the robot's heading in degrees, from -180 to 180
	 */
	public double getHeading() {
		return getPose().getRotation().getDegrees();
	}
	// @Override //TODO: add motor safty back
	// public void stopMotor() {
	// // TODO: Fix stop hack
	// m_frontLeft.setDesiredState(new SwerveModuleState(0,
	// Rotation2d.fromDegrees(45)));
	// m_frontRight.setDesiredState(new SwerveModuleState(0,
	// Rotation2d.fromDegrees(-45)));
	// m_rearLeft.setDesiredState(new SwerveModuleState(0,
	// Rotation2d.fromDegrees(-45)));
	// m_rearRight.setDesiredState(new SwerveModuleState(0,
	// Rotation2d.fromDegrees(45)));
	// }

	// @Override
	// public String getDescription() {
	// // throw new UnsupportedOperationException("Unimplemented method
	// // 'getDescription'");
	// return "swerve";
	// }

	/**
	 * Method to drive the robot using joystick info.
	 *
	 * @param xSpeed
	 *            Speed of the robot in the x direction (forward).
	 * @param ySpeed
	 *            Speed of the robot in the y direction (sideways).
	 * @param rot
	 *            Angular rate of the robot.
	 * @param fieldRelative
	 *            Whether the provided x and y speeds are relative to the field.
	 * @param rateLimit
	 *            Whether to enable rate limiting for smoother control.
	 */
	public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {

		// add rate limiting math to wheels if desired
		double xSpeedCommanded;
		double ySpeedCommanded;
		if (rateLimit) {
			doRateLimitMath(xSpeed, ySpeed);
			xSpeedCommanded = getXRateLimit();
			ySpeedCommanded = getYRateLimit();
			m_currentRotation = m_rotLimiter.calculate(rot);
		} else {
			xSpeedCommanded = xSpeed;
			ySpeedCommanded = ySpeed;
			m_currentRotation = rot;
		}

		// Convert the commanded speeds into the correct units for the drivetrain
		double xSpeedDelivered = xSpeedCommanded * Constants.SwerveConstants.kMaxSpeedMetersPerSecond;
		double ySpeedDelivered = ySpeedCommanded * Constants.SwerveConstants.kMaxSpeedMetersPerSecond;
		double rotDelivered = m_currentRotation * Constants.SwerveConstants.kMaxAngularSpeed;

		// TODO: Rndo Andrei's crime
		// If wheel speeds are 0, set X wheel configuration
		// if (xSpeedDelivered == 0 && ySpeedDelivered == 0 && rotDelivered == 0) {
		// setX();
		// }

		// Calculate new module values depending if using field-relative control
		ChassisSpeeds updatedChassisSpeeds;
		if (fieldRelative) {
			var tmp = DriverStation.getAlliance();
			double flipAngle = tmp.isPresent() && tmp.get() == Alliance.Blue ? 0 : 180;
			updatedChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
					getPose().getRotation().minus(Rotation2d.fromDegrees(flipAngle)));
		} else {
			updatedChassisSpeeds = new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);
		}
		SwerveModuleState[] swerveModuleStates = Constants.SwerveConstants.kDriveKinematics
				.toSwerveModuleStates(updatedChassisSpeeds);

		// Limit the wheel speeds to the maximum speed
		SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,
				Constants.SwerveConstants.kMaxSpeedMetersPerSecond);

		// Set the desired module states
		mapDesiredModuleStates(swerveModuleStates, rotDelivered);

		// feedWatchdog(); // Make motor MotorSafety.feed() Happy
		// Update visualizer
		// log("targetStates", new SwerveModuleState[]{m_frontLeft.getDesiredState(),
		// m_frontRight.getDesiredState(),
		// m_rearLeft.getDesiredState(), m_rearRight.getDesiredState()});
		// log("realStates", new SwerveModuleState[]{m_frontLeft.getState(),
		// m_frontRight.getState(),
		// m_rearLeft.getState(), m_rearRight.getState()});
		m_visualizer.update(m_frontLeft.getState().angle, m_frontRight.getState().angle, m_rearLeft.getState().angle,
				m_rearRight.getState().angle, getPose());
	}

	/**
	 * Sets the wheels into an X formation to prevent movement.
	 */
	public void setX() {

		m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
		m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
		m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
		m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
		// feedWatchdog(); // Make motor MotorSafety.feed() Happy
	}

	private double getXRateLimit() {
		return m_currentTranslationMag * Math.cos(m_currentTranslationDir);
	}

	private double getYRateLimit() {
		return m_currentTranslationMag * Math.sin(m_currentTranslationDir);
	}

	private void doRateLimitMath(double xSpeed, double ySpeed) {
		// Convert XY to polar for rate limiting
		double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
		double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

		// Calculate the direction slew rate based on an estimate of the lateral
		// acceleration
		double directionSlewRate;
		if (m_currentTranslationMag != 0.0) {
			directionSlewRate = Math.abs(SwerveConstants.kDirectionSlewRate / m_currentTranslationMag);
		} else {
			directionSlewRate = 500.0; // some high number that means the slew rate is effectively instantaneous
		}

		double currentTime = WPIUtilJNI.now() * 1e-6;
		double elapsedTime = currentTime - m_prevTime;
		double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
		if (angleDif < 0.45 * Math.PI) {
			m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
					directionSlewRate * elapsedTime);
			m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
		} else if (angleDif > 0.85 * Math.PI) {
			if (m_currentTranslationMag > 1e-4) { // some small number to avoid floating-point errors with equality
													// checking
				// keep currentTranslationDir unchanged
				m_currentTranslationMag = m_magLimiter.calculate(0.0);
			} else {
				m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
				m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
			}
		} else {
			m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
					directionSlewRate * elapsedTime);
			m_currentTranslationMag = m_magLimiter.calculate(0.0);
		}
		m_prevTime = currentTime;
	}

	private void mapDesiredModuleStates(SwerveModuleState[] swerveModuleStates, double rotDelivered) {
		m_frontLeft.setDesiredState(swerveModuleStates[0]);
		m_frontRight.setDesiredState(swerveModuleStates[1]);
		m_rearLeft.setDesiredState(swerveModuleStates[2]);
		m_rearRight.setDesiredState(swerveModuleStates[3]);
	}

	/**
	 * Returns the currently-estimated pose of the robot.
	 *
	 * @return The pose.
	 */
	public Pose2d getPose() {
		return m_odometry.getPoseMeters();
	}

	// getter for m_odometry
	public SwerveDriveOdometry getOdometry() {
		return m_odometry;
	}

	public void updateOdometry() {
		m_odometry.update(getRawGyro(), new SwerveModulePosition[]{m_frontLeft.getPosition(),
				m_frontRight.getPosition(), m_rearLeft.getPosition(), m_rearRight.getPosition()});
	}

	public ChassisSpeeds getChassisSpeeds() {
		return SwerveConstants.kDriveKinematics.toChassisSpeeds(m_frontLeft.getState(), m_frontRight.getState(),
				m_rearLeft.getState(), m_rearRight.getState());
	}

	/**
	 * Resets the odometry to the specified pose.
	 *
	 * @param pose
	 *            The pose to which to set the odometry.
	 */
	public void resetOdometry(Pose2d pose) {
		m_odometry.resetPosition(getRawGyro(), new SwerveModulePosition[]{m_frontLeft.getPosition(),
				m_frontRight.getPosition(), m_rearLeft.getPosition(), m_rearRight.getPosition()}, pose);
	}

}
