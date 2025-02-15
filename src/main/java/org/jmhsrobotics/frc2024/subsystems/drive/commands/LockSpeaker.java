package org.jmhsrobotics.frc2024.subsystems.drive.commands;

import org.jmhsrobotics.frc2024.subsystems.drive.DriveSubsystem;
import org.jmhsrobotics.frc2024.subsystems.vision.VisionSubsystem;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

//TODO: move all hardcoded numbers to constants file
public class LockSpeaker extends Command {
	private DriveSubsystem drive;
	private VisionSubsystem vision;

	private PIDController lockPID;

	private double angleGoal;
	private double fiducialID = -1;
	private Pose2d lastAprilTag;

	public LockSpeaker(DriveSubsystem drive, VisionSubsystem vision) {
		this.drive = drive;
		this.vision = vision;
		this.lockPID = new PIDController(0.01, 0, 0);
		SmartDashboard.putData("LockPID", lockPID);

		this.angleGoal = 180;

		// this.angleGoal = this.target.getYaw();
		// TODO: FIXME: move to the initialize method, lastApriltag is not reset between
		// command runs and could cause issues when running the command more then one
		// time.

		addRequirements(this.drive); // TODO: Figure out how to deal with vision requirements
	}

	@Override
	public void initialize() {
		this.lockPID.reset();
		this.lockPID.setSetpoint(this.angleGoal);
		// SmartDashboard.putData(this.lockPID);
		this.lockPID.setTolerance(5, 1);
		this.lockPID.enableContinuousInput(-180, 180);
		// TODO: add portential april estimation
		// var potentialAprilTag =
		// this.vision.getAprilTagLayout().getTagPose(this.fiducialID);
		// if (potentialAprilTag.isPresent()) {
		// Pose3d estimatedLocation = potentialAprilTag.get();
		// this.lastAprilTag = estimatedLocation.toPose2d();
		// }

		// our goal should be 0 degrees if the camera is in the center of the robot
		// Right now we are not accounting for the camera angle and cordnate sys
		var optionalColor = DriverStation.getAlliance();
		if (optionalColor.isPresent()) {
			this.fiducialID = optionalColor.get() == Alliance.Blue ? 7 : 4;
		}
		this.drive.stopDrive();

	}

	@Override
	public void execute() {
		PhotonTrackedTarget aprilTag = this.vision.getTarget(fiducialID);

		if (aprilTag != null) {
			this.lastAprilTag = this.vision.targetToField(aprilTag.getBestCameraToTarget(), this.drive.getPose())
					.toPose2d();

		}
		if (this.lastAprilTag != null) {
			Transform2d transform = this.lastAprilTag.minus(this.drive.getPose());
			double theta = Math.toDegrees(Math.atan2(transform.getY(), transform.getX()));
			// SmartDashboard.putNumber("Theta", theta);

			var rawOutput = this.lockPID.calculate(theta);
			double output = MathUtil.clamp(rawOutput, -1, 1);
			// TODO: Remove smart dashboard values

			// SmartDashboard.putNumber("Output", output);
			// SmartDashboard.putNumber("mes", theta);

			// TODO: flip the output sign
			// Figured out the issue it appears that we had some values flipped and it was
			// causing the pid loop not to output a continous output rather a stepped output
			this.drive.drive(0, 0, -output, true, true);

			// SmartDashboard.putNumber("LockPID/PositionError",
			// this.lockPID.getPositionError());
			// SmartDashboard.putNumber("LockPID/VelocityError",
			// this.lockPID.getVelocityError());
			// SmartDashboard.putNumber("LockPID/output", output);
			// SmartDashboard.putNumber("LockPID/currentYaw", currentYaw);

		} else {
			this.drive.drive(0, 0, 0, true, true);
		}

	}

	@Override
	public boolean isFinished() {
		return this.lockPID.atSetpoint();
		// return false;
	}

	@Override
	public void end(boolean interrupted) {
		this.drive.drive(0, 0, 0, true, false);
	}
}
