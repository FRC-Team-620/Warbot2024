// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.jmhsrobotics.frc2024.subsystems.drive.commands;

import org.jmhsrobotics.frc2024.Constants;
import org.jmhsrobotics.frc2024.controlBoard.ControlBoard;
import org.jmhsrobotics.frc2024.subsystems.drive.DriveSubsystem;
import org.jmhsrobotics.frc2024.subsystems.vision.VisionSubsystem;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveCommand extends Command {
	/** Creates a new DriveCommand. */
	private final DriveSubsystem driveSubsystem;
	private final VisionSubsystem visionSubsystem;
	private final ControlBoard control;

	// Angle lock
	private final PIDController lockPID;

	private final double angleGoal = 180;
	private double fiducialID = -1;
	private Pose2d lastAprilTag;

	public DriveCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, ControlBoard control) {
		// Use addRequirements() here to declare subsystem dependencies.

		this.driveSubsystem = driveSubsystem;
		this.visionSubsystem = visionSubsystem;
		this.control = control;

		this.lockPID = new PIDController(0.0125, 0, 0.0001);

		addRequirements(this.driveSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		this.lockPID.reset();
		this.lockPID.setSetpoint(this.angleGoal);
		// SmartDashboard.putData(this.lockPID);
		// this.lockPID.setTolerance(3, 1);
		this.lockPID.enableContinuousInput(-180, 180);
		var optionalColor = DriverStation.getAlliance();
		if (optionalColor.isPresent()) {
			this.fiducialID = optionalColor.get() == Alliance.Blue ? 7 : 4;
		}
		// stop drive is a method to set speed inputs to zero and angle the wheels in
		// brake angle
		this.driveSubsystem.stopDrive();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		//
		double xSpeed = MathUtil.applyDeadband(
				this.getSquareInput(-this.control.yInput()) * Constants.SwerveConstants.kMaxXSpeed,
				Constants.SwerveConstants.kDeadBand);
		double ySpeed = MathUtil.applyDeadband(
				this.getSquareInput(-this.control.xInput()) * Constants.SwerveConstants.kMaxYSpeed,
				Constants.SwerveConstants.kDeadBand);
		double rotationSpeed = MathUtil.applyDeadband(
				this.getSquareInput(-this.control.rotationalInput()) * Constants.SwerveConstants.kMaxRotationSpeed,
				Constants.SwerveConstants.kDeadBand);
		// SmartDashboard.putNumber("SwerveDrive/Input/SwerveDriveXSpeed", xSpeed);
		// SmartDashboard.putNumber("SwerveDrive/Input/SwerveDriveXSpeed", ySpeed);
		// SmartDashboard.putNumber("SwerveDrive/Input/SwerveDriveXSpeed",
		// rotationSpeed);
		// SmartDashboard.putNumber("Angle Offset", computeAngleLockValue());

		if (this.control.AprilLockOn().getAsBoolean()) {
			rotationSpeed += computeAngleLockValue();
		}
		this.driveSubsystem.drive(xSpeed, ySpeed, rotationSpeed, Constants.SwerveConstants.kFieldRelative,
				Constants.SwerveConstants.kRateLimit);

		// If a/b is pressed on the contorller, set the robot to brake mode/ re-define
		// forward respecitively

		// control.brake() returns a trigger value(not a boolean), so use .getAsBoolean
		// to convert it to a boolean(same for setZeroHeading)
		if (this.control.brake().getAsBoolean()) {
			this.driveSubsystem.brake();
		} else if (this.control.setZeroHeading().getAsBoolean()) {
			// zero heading is a method in driveSubsystem that "redefine" forward for the
			// robot
			this.driveSubsystem.zeroHeading();
		}
	}

	private double computeAngleLockValue() {
		double out = 0;
		PhotonTrackedTarget aprilTag = this.visionSubsystem.getTarget(fiducialID);

		if (aprilTag != null) {
			this.lastAprilTag = this.visionSubsystem
					.targetToField(aprilTag.getBestCameraToTarget(), this.driveSubsystem.getPose()).toPose2d();

		}
		if (this.lastAprilTag != null) {
			Transform2d transform = this.lastAprilTag.minus(this.driveSubsystem.getPose());
			double theta = Math.toDegrees(Math.atan2(transform.getY(), transform.getX()));
			// SmartDashboard.putNumber("Theta", theta);

			var rawOutput = this.lockPID.calculate(theta);
			double output = MathUtil.clamp(rawOutput, -1, 1);

			out = -output;

		} else {
			out = 0;
		}
		return out;
	}
	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		this.driveSubsystem.stopDrive();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}

	// This method square the input(for less sensitive control)
	private double getSquareInput(double input) {
		return Math.pow(input, 2) * Math.signum(input);
	}
	public boolean lockAtGoal() {
		return this.lockPID.atSetpoint();
	}

}
