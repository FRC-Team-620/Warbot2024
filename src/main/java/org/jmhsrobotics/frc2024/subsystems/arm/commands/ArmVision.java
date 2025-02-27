package org.jmhsrobotics.frc2024.subsystems.arm.commands;

import org.jmhsrobotics.frc2024.subsystems.arm.ArmPIDSubsystem;
import org.jmhsrobotics.frc2024.subsystems.drive.DriveSubsystem;
import org.jmhsrobotics.frc2024.subsystems.vision.VisionSubsystem;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

//TODO: move all hardcoded numbers to constants file
public class ArmVision extends Command {
	private ArmPIDSubsystem arm;
	private VisionSubsystem vision;
	private DriveSubsystem drive;
	private double fiducialID = -1;

	private Pose2d lastAprilTag;
	private InterpolatingDoubleTreeMap armAngles = new InterpolatingDoubleTreeMap();

	/**
	 * Moves Arm into the corect angle/height based off of the distance to the
	 * speaker apriltag. Command Never ends
	 *
	 * @param arm
	 * @param vision
	 * @param drive
	 */
	public ArmVision(ArmPIDSubsystem arm, VisionSubsystem vision, DriveSubsystem drive) {
		this.arm = arm;
		this.vision = vision;
		armAngles.put(0d, 0d);
		// armAngles.put(1.49d, 10d);
		// armAngles.put(4d, 31d);
		// armAngles.put(2.55d, 25d);
		// armAngles.put(3.28d, 28.5d);
		// armAngles.put(4.5d, 32d);
		// armAngles.put(1.76d, 14d);
		// armAngles.put(2.16d, 17.5d);
		// armAngles.put(2.39d, 20.6d);
		// armAngles.put(2.7d, 22d);
		// armAngles.put(3.0d, 23.755d);
		// armAngles.put(3.32d, 25.3d);
		armAngles.put(2.18d, 21.75d);
		armAngles.put(2.42d, 23.5d);
		armAngles.put(2.54d, 24d);
		armAngles.put(3.14d, 27.5d);
		armAngles.put(3.32d, 28d);
		armAngles.put(3.69d, 29d);
		armAngles.put(4d, 30.5d);

		SmartDashboard.putNumber("Armangle", 0);

		this.drive = drive;

		addRequirements(this.arm);
	}

	@Override
	public void initialize() {
		var optionalColor = DriverStation.getAlliance();
		if (optionalColor.isPresent()) {
			this.fiducialID = optionalColor.get() == Alliance.Blue ? 7 : 4;
		}

	}

	@Override
	public void execute() {
		// arm.setGoal(SmartDashboard.getNumber("Armangle", 0));
		PhotonTrackedTarget aprilTag = this.vision.getTarget(fiducialID);
		if (aprilTag != null) {
			this.lastAprilTag = vision.targetToField(aprilTag.getBestCameraToTarget(), drive.getPose()).toPose2d(); // TODO:
																													// Clean
																													// up
			// hack

		}
		if (this.lastAprilTag != null) {
			double dist = lastAprilTag.getTranslation().getDistance(drive.getPose().getTranslation());
			SmartDashboard.putNumber("Distance", dist);

			// arm.setGoal(SmartDashboard.getNumber("ArmAngle", 0));
			double angle = armAngles.get(dist);
			arm.setGoal(angle);

			// Transform2d transform = this.lastAprilTag.minus(this.arm.getPose());
			// double theta = Math.toDegrees(Math.atan2(transform.getY(),
			// transform.getX()));
			// // SmartDashboard.putNumber("Theta", theta);

			// var rawOutput = this.lockPID.calculate(theta);
			// double output = MathUtil.clamp(rawOutput, -0.5, 0.5);

			// // TODO: flip the output sign
			// this.arm.drive(0, 0, output, true, true);

			// SmartDashboard.putNumber("LockPID/PositionError",
			// this.lockPID.getPositionError());
			// SmartDashboard.putNumber("LockPID/VelocityError",
			// this.lockPID.getVelocityError());
			// SmartDashboard.putNumber("LockPID/output", output);
			// SmartDashboard.putNumber("LockPID/currentYaw", currentYaw);

		} else {
			SmartDashboard.putNumber("Distance", -1);
		}

	}

	@Override
	public boolean isFinished() {
		// return false;
		return this.arm.atGoal();
	}

	@Override
	public void end(boolean interrupted) {
		// this.arm.drive(0, 0, 0, true, false);
	}
}
